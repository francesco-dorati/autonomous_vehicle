#include <Arduino.h>
#include <WiFiNINA.h>
#include <math.h>
#include <PID_v1.h>

/*    
    Motor Controller 
    Handles motor control, path following and encoders odometry calculations

    INPUTS (Serial1 115200) 
    ALL INPUTS ENDS WITH '\n'

    "STP" 
        - stop motors

    "PWR <pow_l> <pow_r>" 
        - set target power
        - (pow_l, pow_r) in [-255, 255]

    "VEL <vx> <vt>" 
        - set target velocity
        - (vx, vt) in [mm/s, mrad/s]

    "PID <kp> <ki> <kd>"
        - change pid values

    "ORQ" -> "<x> <y> <theta>"
        - odometry request
        - (x, y) in [mm], theta in [mrad], 
        - (vl, va) in [mm/s, mrad/s]

    "ORS"
        - reset odometry values
    
    "OST <x> <y> <theta>"
        - set odometry values
        - (x, y) in [mm], theta in [mrad]

    "ODB" -> "<x> <y> <theta> <vx> <vy>"
        - get debug odometry values
        - (x, y) in [mm], theta in [mrad], 
        - (vl, va) in [mm/s, mrad/s]


    "PTH <n> <x1> <y1> <th1> ... <xn> <yn> <thn>"
        - set new path to follow
        - n: number of points
        - (x, y) in [mm], th in [mrad]
        - requirements:
            - max number of points simultaneously: 20
            - distance between targets: 100 mm < dist < 1000 mm
            - heading between targets < pi
    
    "APP <n> <x> <y> <th> ... <xn> <yn> <thn>"
        - append point to path
        - n: number of points
        - (x, y) in [mm], th in [mrad]

*/

#define BASE_CONTROL_INTERVAL 20 // ms
#define POSITION_CONTROL_INTERVAL 100 // ms
#define SERIAL_CONTROL_INTERVAL 100 // ms

// STATE
enum State {
    STALL,
    POWER_CONTROL,
    VELOCITY_CONTROL,
    POSITION_CONTROL
};
State control_state = STALL;

// SERIAL
#define END_CHAR '\n'
long last_serial_time = 0;
void handle_serial();

// GENERAL
class Position { // general unit
    public:
        int x, y, th;
        Position();
        Position(int x, int y, int th);
        void reset();
};

// ENCODERS
#define LA_ENCODER_PIN 11 // LEFT GREEN
#define LB_ENCODER_PIN 10 // LEFT YELLOW
#define RA_ENCODER_PIN 9  // RIGHT GREEN
#define RB_ENCODER_PIN 8  // RIGHT YELLOW
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
Position actual_robot_position_u = Position(); // x, y (um), theta (urad)
void left_tick();
void right_tick();
void reset_odometry();
void set_odometry(Position p_u);
void update_odometry();

// POSITION CONTROL
#define QUEUE_SIZE 20
class PositionQueue {
    public:
        PositionQueue();
        Position pop();
        void push(Position p);
        bool is_empty();
        void reset();
    private:
        Position queue[QUEUE_SIZE];
        int front, rear, count;
};
long last_position_control_time = 0;
Position target_robot_position_m = Position(); // x, y (mm), theta (mrad)
PositionQueue target_position_queue = PositionQueue();
void position_control();
void calculate_inverse_kinematic();
// dist
#define DIST_THRESH_MM 25   // mm   distance where target is reached
#define CHANGE_THRESH_MM 50 // mm    distance where target is changed
#define MIN_LIN_SPEED_DIST_MM 100 // mm     distance where minimum speed is reached 
#define MIN_LIN_SPEED_MMS 0         // mm/s    minimum speed
#define MAX_LIN_SPEED_DIST_MM 500 // mm     distance where maximum speed is reached
#define MAX_LIN_SPEED_MMS 0       // mm/s    maximum speed
const int LIN_SPEED_SLOPE = (MAX_LIN_SPEED_MMS - MIN_LIN_SPEED_MMS)/(MAX_LIN_SPEED_DIST_MM - MIN_LIN_SPEED_DIST_MM); 
int distance_error_mm();
int distance_velocity(int dist_mm);
// heading
#define H_THRESH_MRAD 100 // 5 degrees    angle where target is reached
#define MIN_ANG_SPEED_DIST_MRAD 0 // mrad/s  angle where minimum speed is reached
#define MIN_ANG_SPEED_MRADS 0 // mrad/s        minimum speed
#define MAX_ANG_SPEED_DIST_MRAD 0 // mrad/s     angle where maximum speed is reached
#define MAX_ANG_SPEED_MRADS 100 // mrad/s      maximum speed
const int ANG_SPEED_SLOPE = (MAX_ANG_SPEED_MRADS - MIN_ANG_SPEED_MRADS)/(MAX_ANG_SPEED_DIST_MRAD - MIN_ANG_SPEED_DIST_MRAD);
int heading_error_mrad();
int heading_velocity(int heading_mrad);
// alpha
#define A_THRESH_MRAD 200 // 10 degrees
#define ALPHA_SPEED_MRADS 0 // 5 degrees
int alpha_error_mrad();
int alpha_velocity(int alpha_mrad);


// VELOCITY CONTROL
#define MAX_POW 250
#define MIN_POW 0
int target_robot_velocities_m[2] = {0, 0}; // vx (mm/s), vt (mrad/s)
int target_wheel_velocities_m[2] = {0, 0}; // vl, vr (mm/s)
double pid_goal_left = 0, pid_goal_right = 0; // PID goal
double pid_actual_left = 0, pid_actual_right = 0; // PID actual
double pid_output_left = 0, pid_output_right = 0; // PID output
double Kp = 0, Ki = 0, Kd = 0;
PID PID_LEFT(&pid_actual_left, &pid_output_left, &pid_goal_left, Kp, Ki, Kd, DIRECT);
PID PID_RIGHT(&pid_actual_right, &pid_output_right, &pid_goal_right, Kp, Ki, Kd, DIRECT);
void velocity_control();


// POWER CONTROL
#define ENA 4   // BROWN
#define IN1 2   // RED
#define IN2 3   // ORANGE
#define ENB 5   // BLUE
#define IN3 6   // YELLOW
#define IN4 7   // GREEN
int target_powers[2] = {0, 0}; // left, right
void send_motor_powers();

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
    attachInterrupt(digitalPinToInterrupt(RA_ENCODER_PIN), right_tick, RISING);
    attachInterrupt(digitalPinToInterrupt(LA_ENCODER_PIN), left_tick, RISING);

    // PID
    PID_LEFT.SetMode(AUTOMATIC);
    PID_LEFT.SetOutputLimits(-MAX_POW, MAX_POW);
    PID_LEFT.SetSampleTime(BASE_CONTROL_INTERVAL);
    PID_RIGHT.SetMode(AUTOMATIC);
    PID_RIGHT.SetOutputLimits(-MAX_POW, MAX_POW);
    PID_RIGHT.SetSampleTime(BASE_CONTROL_INTERVAL);

    stop_motors();
    reset_odometry();
}


//// LOOP ////
void loop() {
    long t_start = millis();

    // handle serial commands
    if (millis() - last_serial_time >= SERIAL_CONTROL_INTERVAL) {
        handle_serial();
        last_serial_time = millis();
    }

    // update odometry
    update_odometry();

    if (control_state != STALL) {
        // position control
        if (control_state == POSITION_CONTROL && (millis() - last_position_control_time) >= POSITION_CONTROL_INTERVAL) {
            // calculate target robot velocities
            position_control();
            calculate_inverse_kinematic(); // calculate target wheels velocity
            last_position_control_time = millis();
        }

        if (control_state == POSITION_CONTROL || control_state == VELOCITY_CONTROL) {
            // compute motor powers
            velocity_control();
        }

        if (target_powers[0] == 0 && target_powers[1] == 0) { stop_motors(); }

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
    target_robot_velocities_m[0] = 0;
    target_robot_velocities_m[1] = 0;
    target_robot_position_m.reset();
    target_position_queue.reset();
}

long normalize_angle_urad(long angle) {
    return ((angle + (int) PI_URAD) % (int) TWO_PI_URAD) - PI_URAD;
}
long normalize_angle_mrad(long angle) {
    return ((angle + (int) PI_MRAD) % (int) TWO_PI_MRAD) - PI_MRAD;
}

Position::Position() {
    this->x = 0;
    this->y = 0;
    this->th = 0;
}
Position::Position(int x, int y, int th) {
    this->x = x;
    this->y = y;
    this->th = th;
}
void Position::reset() {
    this->x = 0;
    this->y = 0;
    this->th = 0;
}

// SERIAL
void handle_serial() {
    while (Serial1.available() >= 3) {
        char command[4];
        command[3] = '\0';
        Serial1.readBytes(command, 3);

        if (strcmp(command, "STP") == 0) {
            // stop motors
            // "STP"
            stop_motors();

        } else if (strcmp(command, "PWR") == 0) {
            // power control
            // "PWR <pow_l> <pow_r>" 
            // if (Serial1.available() < 2) return;
            target_powers[0] = Serial1.parseInt();
            target_powers[1] = Serial1.parseInt();
            control_state = POWER_CONTROL;

        } else if (strcmp(command, "VEL") == 0) {
            // velocity control
            // "VEL <vx> <vt>" 
            // if (Serial1.available() < 3) return;
            target_robot_velocities_m[0] = Serial1.parseInt();
            target_robot_velocities_m[1] = Serial1.parseInt();
            calculate_inverse_kinematic();
            control_state = VELOCITY_CONTROL;

        } else if (strcmp(command, "PID") == 0) {
            // pid values change
            // "PID <kp> <ki> <kd>"
            // if (Serial1.available() < 6) return;
            Kp = Serial1.parseFloat();
            Ki = Serial1.parseFloat();
            Kd = Serial1.parseFloat();
            PID_LEFT.SetTunings(Kp, Ki, Kd);
            PID_RIGHT.SetTunings(Kp, Ki, Kd);

        } else if (strcmp(command, "ORQ") == 0) {
            // odometry request
            // "ORQ" -> "<x> <y> <theta>"
            Serial1.print(actual_robot_position_u.x/1000.0); // x
            Serial1.print(" ");
            Serial1.print(actual_robot_position_u.y/1000.0); // y
            Serial1.print(" ");
            Serial1.println(actual_robot_position_u.th/1000.0); // theta

        } else if (strcmp(command, "ORS") == 0) {
            // odometry reset
            // "ORS"
            reset_odometry();

        } else if (strcmp(command, "OST") == 0) {
            // odometry set
            // "OST <x> <y> <theta>"
            int x_u = Serial1.parseInt()*1000; 
            int y_u = Serial1.parseInt()*1000; 
            int th_u = Serial1.parseInt()*1000; 
            set_odometry(Position(x_u, y_u, th_u));

        } else if (strcmp(command, "ODB") == 0) {
            // odometry debug
            // "ORQ" -> "<x> <y> <theta> <vl> <va>"
            Serial1.print(actual_robot_position_u.x/1000.0); // x
            Serial1.print(" ");
            Serial1.print(actual_robot_position_u.y/1000.0); // y
            Serial1.print(" ");
            Serial1.println(actual_robot_position_u.th/1000.0); // theta
            Serial1.print(" ");
            Serial1.print(actual_robot_velocities_m[0]); // vx
            Serial1.print(" ");
            Serial1.println(actual_robot_velocities_m[1]); // vt

        } else if (strcmp(command, "PTH") == 0) {
            // set path to follow
            // "PTH <n> <x1> <y1> <th1> ... <xn> <yn> <thn>"
            target_position_queue.reset();
            int n = Serial1.parseInt();
            for (int i = 0; i < n; i++) {
                long x_mm = Serial1.parseInt();
                long y_mm = Serial1.parseInt();
                long th_mrad = Serial1.parseInt();
                target_position_queue.push(Position(x_mm, y_mm, th_mrad));
            }

        } else if (strcmp(command, "APP") == 0) {
            int n = Serial1.parseInt();
            for (int i = 0; i < n; i++) {
                long x_mm = Serial1.parseInt();
                long y_mm = Serial1.parseInt();
                long th_mrad = Serial1.parseInt();
                target_position_queue.push(Position(x_mm, y_mm, th_mrad));
            }

        } else {
            // invalid command
            // ignore
        }

        // read until END
        while (Serial1.available() > 0 && Serial1.read() != END_CHAR) {};
    }
}

// ODOMETRY
void left_tick() { 
    if (digitalRead(LB_ENCODER_PIN) == HIGH) ticks_l++;  // Forward
    else ticks_l--;  // Backward
}
void right_tick() {
    if (digitalRead(RB_ENCODER_PIN) == HIGH) ticks_r++;  // Forward
    else ticks_r--;  // Backward
} 
void reset_odometry() {
    ticks_l = 0;
    ticks_r = 0;
    prev_ticks_l = 0;
    prev_ticks_r = 0;
    prev_time_encoder_us = micros();
    actual_wheel_velocities_m[0] = 0;
    actual_wheel_velocities_m[1] = 0;
    actual_robot_velocities_m[0] = 0; 
    actual_robot_velocities_m[1] = 0;
    actual_robot_position_u.reset();
    target_robot_position_m.reset();
    target_position_queue.reset();

}
void set_odometry(Position p_u) {
    actual_robot_position_u = p_u;
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
    actual_robot_position_u.th += dT_urad; // urad
    // normalize theta
    actual_robot_position_u.th = normalize_angle_urad(actual_robot_position_u.th);
    // update x, y
    actual_robot_position_u.x += dS_um * cos(actual_robot_position_u.th/1000000.0); // um
    actual_robot_position_u.y += dS_um * sin(actual_robot_position_u.th/1000000.0); // um

    // DEBUG
    // if (Serial) {
    //     Serial.print("TICKS TOT "); Serial.println(ticks_r); Serial.print("Ticks: L "); Serial.print(d_ticks_l); Serial.print(", R"); Serial.println(d_ticks_r);

    //     Serial.print("Time: "); Serial.print(d_time_us/1000); Serial.println("ms");

    //     Serial.print("dL "); Serial.print(dL_um); Serial.print(", dR "); Serial.println(dR_um);

    //     Serial.print(" dS ");Serial.print(dS_um);Serial.print(", dT ");Serial.println(dT_urad);

    //     Serial.print("Wheels (mm/s): L ");Serial.print(wheel_velocities[0]);Serial.print(", R");Serial.println(wheel_velocities[1]);

    //     Serial.print("Velocity: VX ");Serial.print(robot_velocities[0]);Serial.print(", VTHETA (mrad/s) ");Serial.println(robot_velocities[1]);

    //     Serial.print("Position (mm): X "); Serial.print(robot_position[0]/1000.0); Serial.print(", Y "); Serial.print(robot_position[1]/1000.0); Serial.print(", TH "); Serial.println(robot_position[2]/1000.0);
    // }
}

// POSITION CONTROL
PositionQueue::PositionQueue() { 
    front = 0; rear = 0; count = 0;
}
Position PositionQueue::pop() {
    /*
        Returns the next point to follow
        requires: !is_empty()
    */
    if (this->is_empty()) return Position();
    Position p = queue[front];
    front = (front + 1) % QUEUE_SIZE;
    count--;
    return p;
}
void PositionQueue::push(Position p) {
    /*
        Adds a new point to the queue
        requires: count < QUEUE_SIZE
    */
    if (count == QUEUE_SIZE) return;
    queue[rear] = p;
    rear = (rear + 1) % QUEUE_SIZE;
    count++;
}
bool PositionQueue::is_empty() {
    return count == 0;
}
void PositionQueue::reset() {
    front = 0; rear = 0; count = 0;
}
void position_control() {
    /*
        Computes the target velocities based on the target position
        reads: target_robot_position_m[3]
        writes: target_robot_velocities_m[2]
        requires: distance between targets > CHANGE THRESH  
                  heading between targets: 0 < heading < pi
    */
    // calculate distance 
    int dist_mm = distance_error_mm();
    if (dist_mm < CHANGE_THRESH_MM) { // inside change threshold
        if (target_position_queue.is_empty()) { // no more points to follow
            // adjust heading and distance difference
            int alpha_mrad = alpha_error_mrad();
            bool distance_ok = dist_mm < DIST_THRESH_MM;
            bool heading_ok = abs(alpha_mrad) < H_THRESH_MRAD;
            // if position reached
            if (distance_ok && heading_ok) { 
                stop_motors();
                return;
            }

            // adjust distance and heading
            target_robot_velocities_m[0] = distance_ok ? 0 : distance_velocity(dist_mm);
            target_robot_velocities_m[1] = heading_ok ? 0 : alpha_velocity(alpha_mrad);
            return;
        }

        // follow next point
        target_robot_position_m = target_position_queue.pop();
        dist_mm = distance_error_mm();
   } 

    // calculate heading 
    int heading_mrad = heading_error_mrad();
    target_robot_velocities_m[0] = distance_velocity(dist_mm);
    target_robot_velocities_m[1] = heading_velocity(heading_mrad);
    return;
}

int distance_error_mm() {
    /*
        Returns the distance error (mm) between target and actual position
        reads: target_robot_position_m, actual_robot_position_u
    */
   int dx_mm = target_robot_position_m.x - (actual_robot_position_u.x/1000);
   int dy_mm = target_robot_position_m.y - (actual_robot_position_u.y/1000);
   return sqrt(dx_mm*dx_mm + dy_mm*dy_mm);
}
int heading_error_mrad() {
    /*
        Returns the angle between target and actual position (mrad)
        reads: target_robot_position_m, actual_robot_position_u
    */
    int dx_mm = target_robot_position_m.x - (actual_robot_position_u.x/1000);
    int dy_mm = target_robot_position_m.y - (actual_robot_position_u.y/1000);
    return normalize_angle_mrad(atan2(dy_mm, dx_mm) * 1000);
    // return normalize_angle_mrad(target_robot_position_m[2] - actual_robot_position_u[2]/1000);
}
int alpha_error_mrad() {
    /*
        Returns the heading difference (mrad) between target and actual position
        reads: target_robot_position_m, actual_robot_position_u
    */
    return normalize_angle_mrad(target_robot_position_m.th - actual_robot_position_u.th/1000);
}

int distance_velocity(int dist_mm) {
    /*
        Returns the target velocity based on the distance error
        reads: d_mm
        returns:
            0                                           dist < TRESHOLD
            slope * (dist - MIN_DIST) + MIN_VEL         where MIN_DIST < dist < MAX_DIST
            staturated at MIN/MAX                       elsewhere
    */
    if (dist_mm < DIST_THRESH_MM) return 0;
    if (dist_mm < MIN_LIN_SPEED_DIST_MM) return MIN_LIN_SPEED_MMS;
    if (dist_mm > MAX_LIN_SPEED_DIST_MM) return MAX_LIN_SPEED_MMS;
    return (dist_mm - MIN_LIN_SPEED_DIST_MM) * LIN_SPEED_SLOPE + MIN_LIN_SPEED_MMS;
}
int heading_velocity(int heading_mrad) {
    /*
        Returns the target angular velocity based on the heading error
        reads: heading_mrad
        returns:
            0                                           heading < TRESHOLD
            slope * (heading - MIN_ANG) + MIN_VEL        where MIN_ANG < heading < MAX_ANG
            staturated at MIN/MAX                       elsewhere
    */
    if (abs(heading_mrad) < H_THRESH_MRAD) return 0;
    if (abs(heading_mrad) < MIN_ANG_SPEED_DIST_MRAD) return MIN_ANG_SPEED_MRADS;
    if (abs(heading_mrad) > MAX_ANG_SPEED_DIST_MRAD) return MAX_ANG_SPEED_MRADS;
    return (heading_mrad - MIN_ANG_SPEED_DIST_MRAD) * ANG_SPEED_SLOPE + MIN_ANG_SPEED_MRADS;
}
int alpha_velocity(int alpha_mrad) {
    /*
        Returns the target angular velocity based on the heading difference
        reads: alpha_mrad
        fixed speed cut of at A_THRESH_MRAD
    */
    return (abs(alpha_mrad) < A_THRESH_MRAD) ? 0 : ALPHA_SPEED_MRADS; 
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

// VELOCITY CONTROL
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

// POWER CONTROL
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