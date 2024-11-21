#include <Arduino.h>
#include <WiFiNINA.h>
#include <math.h>
#include <PID_v1.h>


/*
    REQUIREMENTS    
    distance between targets: 100 mm < dist < 1000 mm
    heading between targets: 0 < heading < pi

    INPUTS (Serial1 115200)
    "STP" 
        - stop motors

    "PWR <pow_l> <pow_r>" 
        - set target power
        - (pow_l, pow_r) in [-255, 255]

    "VEL <vx> <vt>" 
        - set target velocity
        - (vx, vt) in [mm/s, mrad/s]

    "PID <kpl> <kdl> <kil> <kpr> <kdr> <kir>"
        - change pid values

    "ORQ" -> "ODM <odometry>"
        - odometry request

    "ORS"
        - reset odometry values
    
    "OST <odometry>"
        - set odometry values

    "PTH <n> <x1> <y1> <th1> ... <xn> <yn> <thn>"
        - set path to follow
        - n: number of points
        - (x, y) in [mm], th in [mrad]
    
    "APP <x> <y> <th>"
        - append point to path
        - (x, y) in [mm], th in [mrad]

*/

#define BASE_CONTROL_INTERVAL 20 // ms
#define POSITION_CONTROL_INTERVAL 100 // ms
#define SERIAL_CONTROL_INTERVAL 100 // ms

// MOTORS
#define ENA 0
#define IN1 0
#define IN2 0
#define ENB 0
#define IN3 0
#define IN4 0

// SERIAL
long last_serial_time = 0;

// ENCODERS
#define LA_ENCODER_PIN 0
#define LB_ENCODER_PIN 0
#define RA_ENCODER_PIN 0
#define RB_ENCODER_PIN 0
#define WHEEL_RADIUS_MM 34      // mm
#define WHEEL_CIRCUMFERENCE_UM 213628   // um  
#define WHEEL_DISTANCE_MM 240   // mm
#define COUNTS_PER_REV 370
#define TWO_PI_URAD 6283185     // 2π in microradians
#define PI_URAD 3141593.6       // π in microradians
#define TWO_PI_MRAD 6283.1853
#define PI_MRAD 3141.5936
volatile long ticks_l, ticks_r;
long prev_ticks_l, prev_ticks_r, prev_time_encoder_us;
int actual_wheel_velocities_m[2]; // vl, vr (mm/s)
int actual_robot_velocities_m[2]; // vx (mm/s), vt (mrad/s)
long actual_robot_position_u[3]; // x, y (um), theta (urad)

enum control_state = {
    STALL,
    POWER_CONTROL,
    VELOCITY_CONTROL,
    POSITION_CONTROL
};

// POSITION CONTROL
#define D_THRESH_MM 25
#define H_THRESH_MRAD 100 // 5 degrees
#define A_THRESH_MRAD 200 // 10 degrees
long last_position_control_time = 0;
long target_robot_position_m[3] = {0, 0, 0}; // x, y (mm), theta (mrad)
// int points_to_follow = 0;


// VELOCITY CONTROL
#define MAX_POW 250
#define MIN_POW 0
int target_robot_velocities_m[2] = {0, 0}; // vx (mm/s), vt (mrad/s)
int target_wheel_velocities_m[2] = {0, 0}; // vl, vr (mm/s)
double pid_goal_left = 0, pid_goal_right = 0; // PID goal
double pid_actual_left = 0, pid_actual_right = 0; // PID actual
double pid_output_left = 0, pid_output_right = 0; // PID output
double KpL = 0, KiL = 0, KdL = 0;
double KpR = 0, KiR = 0, KdR = 0;
PID PID_LEFT(&pid_actual_left, &pid_output_left, &pid_goal_left, KpL, KiL, KdL, DIRECT);
PID PID_RIGHT(&pid_actual_right, &pid_output_right, &pid_goal_right, KpR, KiR, KdR, DIRECT);


// POWER CONTROL
int target_powers[2] = {0, 0}; // left, right

void setup() {
    Serial.begin(9600);
    Serial1.begin(115200);

    // Left Motor
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    // Right Motor
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Encoders
    pinMode(LA_ENCODER_PIN, INPUT);
    pinMode(LB_ENCODER_PIN, INPUT);
    pinMode(RA_ENCODER_PIN, INPUT);
    pinMode(RB_ENCODER_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(RA_encoder), right_tick, RISING);
    attachInterrupt(digitalPinToInterrupt(LA_encoder), left_tick, RISING);

    // PID
    PID_LEFT.SetMode(AUTOMATIC);
    PID_LEFT.SetOutputLimits(-MAX_POW, MAX_POW);
    PID_LEFT.SetSampleTime(MOTOR_CONTROL_INTERVAL);
    PID_RIGHT.SetMode(AUTOMATIC);
    PID_RIGHT.SetOutputLimits(-MAX_POW, MAX_POW);
    PID_RIGHT.SetSampleTime(MOTOR_CONTROL_INTERVAL);

    stop_motors();
    reset_encoders();
}


void loop() {
    long t_start = millis();

    // handle serial commands
    if (millis() - last_serial_time >= SERIAL_CONTROL_INTERVAL) {
        handle_serial()
        last_serial_time = millis();
    }

    // update odometry
    update_odometry();

    if (control_state != STALL) {
        // position control
        if (control_state == POSITION_CONTROL && (millis() - last_position_control_time) >= POSITION_CONTROL_INTERVAL) {
            // calculate target robot velocities
            position_control() 
            calculate_inverse_kinematic(); // calculate target wheels velocity
            last_position_control_time = millis();
        }

        if (control_state == POSITION_CONTROL || control_state == VELOCITY_CONTROL) {
            // compute motor powers
            velocity_control();
        }

        // send motor powers
        send_motor_powers();
    }

    // delay
    int dt = millis() - t_start;
    if (dt < BASE_CONTROL_INTERVAL) 
        delay(BASE_CONTROL_INTERVAL - dt);
    
}

void stop_motors() {
    control_state = STALL;
    target_powers[0] = 0;
    target_powers[1] = 0;
    send_motor_powers();
}
long normalize_angle_urad(long angle) {
    return ((angle + PI_URAD) % TWO_PI_URAD) - PI_URAD;
}
long normalize_angle_mrad(long angle) {
    return ((angle + PI_MRAD) % TWO_PI_MRAD) - PI_MRAD;
}

// SERIAL
void handle_serial() {
    if (Serial1.available() >= 3) {
        char command[4];
        command[4] = '\0';
        Serial1.readBytes(command, 3);

        if (strcmp(command, "STP") == 0) {
            stop_motors();

        } else if (strcmp(command, "PWR") == 0) {
            // power control
            if (Serial1.available() < 5) return;
            target_powers[0] = Serial1.parseInt();
            target_powers[1] = Serial1.parseInt();
            control_state = POWER_CONTROL;

        } else if (strcmp(command, "VEL") == 0) {
            // velocity control
            if (Serial1.available() < 5) return;
            target_robot_velocities_m[0] = Serial1.parseInt();
            target_robot_velocities_m[1] = Serial1.parseInt();
            calculate_inverse_kinematic();
            control_state = VELOCITY_CONTROL;

        } else if (strcmp(command, "PID") == 0) {
            // odometry request

        } else if (strcmp(command, "ORQ") == 0) {
            // odometry request

        } else if (strcmp(command, "ORS") == 0) {
            // odometry reset
            reset_odometry();

        } else if (strcmp(command, "OST") == 0) {
            // odometry set

        } else if (strcmp(command, "PTH") == 0) {
            // set path to follow
            if (Serial1.available() < 1) return;
            points_to_follow = Serial1.parseInt();

        } else if (strcmp(command, "APP") == 0) {
            // set path to follow
            if (Serial1.available() < 1) return;
            points_to_follow = Serial1.parseInt();
            if (Serial1.available() < 3*points_to_follow) return;
            for (int i = 0; i < points_to_follow; i++) {
                target_robot_position_m[0] = Serial1.parseInt();
                target_robot_position_m[1] = Serial1.parseInt();
                
            }
        // flush input
        }
    }
}

// ENCODERS
void left_tick() { 
    if (digitalRead(LB_encoder) == HIGH) ticks_l++;  // Forward
    else ticks_l--;  // Backward
}
void right_tick() {
    if (digitalRead(RB_encoder) == HIGH) ticks_r++;  // Forward
    else ticks_r--;  // Backward
} 
void reset_odometry() {
    ticks_l = 0;
    ticks_r = 0;
    prev_ticks_l = 0;
    prev_ticks_r = 0;
    prev_time_encoder_us = micros();
    actual_wheel_velocities[0] = 0;
    actual_wheel_velocities[1] = 0;
    actual_robot_velocities[0] = 0; 
    actual_robot_velocities[1] = 0;
    actual_robot_position[0] = 0; 
    actual_robot_position[1] = 0;
    actual_robot_position[2] = 0;
}
void update_odometry() {
    /*
        Updates the robot odometry
        reads: ticks_l, ticks_r
        writes: actual_wheel_velocities_m[2], actual_robot_velocities_m[2], actual_robot_position_u[3]
    */
    // delta ticks
    long d_ticks_l = ticks_l - prev_ticks_l;
    long d_ticks_r = ticks_r - prev_ticks_r;
    prev_ticks_l = ticks_l;
    prev_ticks_r = ticks_r;

    // delta time
    long t_us = micros();
    float d_time_us = (t_us - prev_time_encoder_us);
    prev_time_encoder_us = t_us;

    long dL_um = (d_ticks_l * WHEEL_CIRCUMFERENCE_UM) / COUNTS_PER_REV;
    long dR_um = (d_ticks_r * WHEEL_CIRCUMFERENCE_UM) / COUNTS_PER_REV;

    long dS_um = (dL_um + dR_um) / 2; // um
    long dT_urad = (dR_um - dL_um) * 1000 / WHEEL_DISTANCE_MM; // urad

    // wheel velocities
    // (um*1000)/us = nm/us = mm/s
    actual_wheel_velocities_m[0] = (dL_um * 1000) / d_time_us; 
    actual_wheel_velocities_m[1] = (dR_um * 1000) / d_time_us;

    // robot velocities
    // (um*1000)/us = nm/us = mm/s
    actual_robot_velocities_m[0] = (dS_um * 1000)/ d_time_us; 
    actual_robot_velocities_m[1] = (dT_urad * 1000) / d_time_us;
    
    // robot position
    actual_robot_position_u[2] += dT_urad; // urad
    // normalize theta
    actual_robot_position_u[2] = normalize_angle_urad(actual_robot_position_u[2]);
    // update x, y
    actual_robot_position_u[0] += dS_um * cos(actual_robot_position_u[2]/1000000.0); // um
    actual_robot_position_u[1] += dS_um * sin(actual_robot_position_u[2]/1000000.0); // um

    // DEBUG
    if (Serial) {
        Serial.print("TICKS TOT "); Serial.println(ticks_r); Serial.print("Ticks: L "); Serial.print(d_ticks_l); Serial.print(", R"); Serial.println(d_ticks_r);

        Serial.print("Time: "); Serial.print(d_time_us/1000); Serial.println("ms");

        Serial.print("dL "); Serial.print(dL_um); Serial.print(", dR "); Serial.println(dR_um);

        Serial.print(" dS ");Serial.print(dS_um);Serial.print(", dT ");Serial.println(dT_urad);

        Serial.print("Wheels (mm/s): L ");Serial.print(wheel_velocities[0]);Serial.print(", R");Serial.println(wheel_velocities[1]);

        Serial.print("Velocity: VX ");Serial.print(robot_velocities[0]);Serial.print(", VTHETA (mrad/s) ");Serial.println(robot_velocities[1]);

        Serial.print("Position (mm): X "); Serial.print(robot_position[0]/1000.0); Serial.print(", Y "); Serial.print(robot_position[1]/1000.0); Serial.print(", TH "); Serial.println(robot_position[2]/1000.0);
    }
}

// POSITION CONTROL
void position_control() {
    /*
        Computes the target velocities based on the target position
        reads: target_robot_position_m[3]
        writes: target_robot_velocities_m[2]
        requires: (d_mm < 1000), (|h_mrad| < pi)
    */
   // calculate distance 
   int dx_mm = target_robot_position_m[0] - (actual_robot_position_u[0]/1000);
   int dy_mm = target_robot_position_m[1] - (actual_robot_position_u[1]/1000);
   int d_mm = sqrt(dx_mm*dx_mm + dy_mm*dy_mm);

   if (d_mm < D_THRESH_MM) {
        // inside threshold
        // calculate alpha (heading difference)
        int delta_heading_mrad = target_robot_position_m[2] - actual_robot_position_u[2]/1000;
        int alpha_mrad = normalize_angle_mrad(delta_heading_mrad);
        if (alpha_mrad < A_THRESH_MRAD) {
            // position reached
            // if there are more points to follow
                // exit
            // else
                // continue with next point
    
        } else {
            // calculate target velocity
            target_robot_velocities_m[0] = 0;
            target_robot_velocities_m[1] = alpha_velocity(alpha_mrad);
            return;
        }
   } else {
        // calculate heading 
        int heading_mrad = normalize_angle_mrad(atan2(dy_mm, dx_mm) * 1000);
        target_robot_velocities_m[0] = distance_velocity(d_mm);
        target_robot_velocities_m[1] = heading_velocity(heading_mrad);
        return;
   }

}

void calculate_inverse_kinematic() {
    /*
        Computes the target wheel velocities based on the target robot velocities
        reads: target_robot_velocities_m[2]
        writes: target_wheel_velocities_m[2]
    */
    target_wheel_velocities_m[0] = target_robot_velocities_m[0] - target_robot_velocities_m[1] * WHEEL_DISTANCE_MM / 2;
    target_wheel_velocities_m[1] = target_robot_velocities_m[0] + target_robot_velocities_m[1] * WHEEL_DISTANCE_MM / 2;
}

// MOTOR CONTROL
void velocity_control() {
    /*
        Computes the motor powers based on the target velocities
        (wheels velocities are already calculated)
        reads: target_wheel_velocities_m[2]
        writes: target_powers[2] 
    */
    // update pid variables 
    pid_actual_left = actual_wheel_velocities_m[0];
    pid_actual_right = actual_wheel_velocities_m[1];
    pid_goal_left = target_wheel_velocities_m[0];
    pid_goal_right = target_wheel_velocities_m[1];
    // compute PID
    PID_LEFT.Compute();
    PID_RIGHT.Compute();
    // update target powers
    target_powers[0] = (int) pid_output_left;
    target_powers[1] = (int) pid_output_right;
}


void send_motor_powers() {
    /*  
        Sends the motor powers to the motors
        target_powers[0] -> left motor [-255, 255]
        target_powers[1] -> right motor [-255, 255]
    */

    // saturation
    // left
    if (abs(target_powers[0]) > MAX_POW) target_powers[0] = (target_powers[0] > 0) ? MAX_POW : -MAX_POW;
    if (abs(target_powers[0]) < MIN_POW) target_powers[0] = 0;
    // right
    if (abs(target_powers[1]) > MAX_POW) target_powers[1] = (target_powers[1] > 0) ? MAX_POW : -MAX_POW;
    if (abs(target_powers[1]) < MIN_POW) target_powers[1] = 0;

    // update motors
    // left
    if (target_powers[0] == 0){
        analogWrite(ENB, 0);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
    } else {
        analogWrite(ENB, abs(target_powers[0]));
        digitalWrite(IN3, (target_powers[0] > 0) ? LOW : HIGH);
        digitalWrite(IN4, (target_powers[0] > 0) ? HIGH : LOW);
    }
    // right
    if (target_powers[1] == 0){
        analogWrite(ENA, 0);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
    } else{
        analogWrite(ENA, abs(target_powers[1]));
        digitalWrite(IN1, (target_powers[1] > 0) ? LOW : HIGH);
        digitalWrite(IN2, (target_powers[1] > 0) ? HIGH : LOW);
    }
}