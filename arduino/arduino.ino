#define PWM_MR 5     // BROWN (D5)
#define IN1_MR 9     // RED (D9)
#define IN2_MR 4     // ORANGE (D4)
#define ENCA_MR 3    // GREEN (D3)
#define ENCB_MR 19   // YELLOW (A5)

#define PWM_ML 6     // BLUE (D6)
#define IN1_ML 7     // YELLOW (D7)
#define IN2_ML 8     // GREEN (D8)
#define ENCA_ML 2    // GREEN (D2)
#define ENCB_ML 18   // YELLOW (A4)

#define COUNTS_PER_REV 1495
#define WHEEL_RADIUS 3.4 // cm
#define WHEEL_DISTANCE_FROM_CENTER 12 // cm
// #define KP 12.0
// #define KI .5
// #define KD .01
#define KP 1.0
#define KI .1
#define KD .01

#define CONTROLLER_FREQ 100 // Hz
#define CONTROLLER_UPDATE_TIME (1000/CONTROLLER_FREQ)

#include "Arduino.h"
#include "MotorController.h"
#include <math.h>

/*
GLOBAL VARIABLES:

mode
    - idle
    - running

position
    - x
    - y
    - theta

goal_velocity
    - vx
    - va

actual_velocity
    - vx
    - va

goal_velocity_wheels
    - left
    - right

actual_velocity_wheels
    - left
    - right



LOOP

    if IDLE
        - read serial
        - if "S" -> RUNNING
        - else delay(100)

    if RUNNING
    read serial
        if "E" -> IDLE
        read goal velocities
        calculate goal velocity wheels (inverse kinematics)
        update goal velocity wheels

    update motors

    get current wheels velocities

    odometry calcualtions
        - x, y, theta
        - vx, vt

    if received serial
        -> produce response
        -> send response

    delay to keep loop time constant

*/


enum mode {
    IDLE = 0,
    RUNNING = 1
} ;

struct position {
    double x; // cm
    double y; // cm
    double theta; // degrees
    position(): x(0), y(0), theta(0) {};
    position(double x, double y, double t): x(x), y(y), theta(t) {};
};

struct robot_velocity
{
    double vx;    // cm/s
    double va;  // degrees/s
    robot_velocity(): vx(0), va(0) {};
    robot_velocity(double x, double t): vx(x), va(t) {};
};

struct wheels_velocity {
    double left; // rpm
    double right; // rpm
    wheels_velocity(): left(0), right(0) {};
    wheels_velocity(double l, double r): left(l), right(r) {};
};

mode controller_mode = IDLE;
bool received_serial = false;

position robot_position;
robot_velocity goal_velocity;
robot_velocity actual_velocity;

wheels_velocity goal_velocity_wheels;
wheels_velocity actual_velocity_wheels;

MotorController motor_left(PWM_ML, IN1_ML, IN2_ML, ENCA_ML, ENCB_ML, WHEEL_RADIUS, COUNTS_PER_REV, true, KP, KI, KD);
MotorController motor_right(PWM_MR, IN1_MR, IN2_MR, ENCA_MR, ENCB_MR, WHEEL_RADIUS, COUNTS_PER_REV, false, KP, KI, KD);
//

// long prev_ticks_l = 0;
// long prev_ticks_r = 0;

// float dist[4];

void setup() {
    Serial.begin(115200);
}


void loop() {
    if (controller_mode == IDLE) {
        // IDLE LOOP
        String str = read_serial();
        if (str == "S" || str == "START") { // START RUNNING MODE
            controller_mode = RUNNING;
            Serial.println("OK");
            return;
        } else delay(100);

    } else if (controller_mode == RUNNING) {
        // RUNNING LOOP
        unsigned long t_start = micros();
        received_serial = false;

        // read serial
        String str = read_serial();
        if (str == "E" || str == "EXIT") { // EXIT RUNNING MODE
            controller_mode = IDLE;
            Serial.println("OK");
            return;
        } else if (str.startsWith("V")) { // SET NEW GOAL
            goal_velocity = process_data(str);
            goal_velocity_wheels = inverse_kinematics(goal_velocity);
            received_serial = true;
        } 

        // update motors
        motor_left.update(goal_velocity_wheels.left);
        motor_right.update(goal_velocity_wheels.right);

        // update position and velocity
        robot_position = odometry(robot_position, motor_left.space, motor_right.space);
        actual_velocity_wheels = wheels_velocity(motor_left.velocity_rpm, motor_right.velocity_rpm);
        actual_velocity = forward_kinematics(actual_velocity_wheels);

        // response
        if (received_serial) {
            String res = produce_response(robot_position, actual_velocity, actual_velocity_wheels, (micros() - t_start)/1000);
            Serial.println(res);
        }

        // delay to keep loop time constant
        unsigned long dt = (micros() - t_start)/1000;
        if (dt < CONTROLLER_UPDATE_TIME)
            delay(CONTROLLER_UPDATE_TIME - dt);
    }

}


// void loop() {
//     // read serial data
//     unsigned long t_start = micros();
//     String d = read_serial();

//     if (controller_mode == IDLE) {
//         if (d == "S") {
//             controller_mode = RUNNING;
//             Serial.println("OK");
//             return;
//         } else delay(100);

//     } else if (controller_mode == RUNNING) {
//         received_serial = false;

//         if (d == "E") {
//             controller_mode = IDLE;
//             Serial.println("OK");
//             return;

//         } else if (d.startsWith("V")) {
//             goal_velocity = process_data(d);
//             goal_velocity_wheels = inverse_kinematics(goal_velocity);
//             received_serial = true;
//         } 

//         motor_left.update(goal_velocity_wheels.left);
//         motor_right.update(goal_velocity_wheels.right);

//         if (received_serial) {
//             int d_ticks_l = motor_left.ticks() - prev_ticks_l;
//             int d_ticks_r = motor_right.ticks() - prev_ticks_r;
//             prev_ticks_l += d_ticks_l;
//             prev_ticks_r += d_ticks_r;

//             String res = produce_response(motor_left, motor_right, d_ticks_l, d_ticks_r, dist, (micros() - t_start));
//             Serial.println(res);
//         }

//         unsigned long dt = (micros() - t_start)/1000;
//         if (dt < CONTROLLER_UPDATE_TIME)
//             delay(CONTROLLER_UPDATE_TIME - dt);

//     }
// }
 
String read_serial() {
    String s = "";
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') break;
        s += c;
    };
    return s;
}

robot_velocity process_data(String s) {
    int sp1 = s.indexOf(' ');
    int sp2 = s.indexOf(' ', sp1 + 1);
    if (sp1 != -1 && sp2 != -1) {
        double vx = s.substring(sp1 + 1, sp2).toDouble();
        double va = s.substring(sp2 + 1).toDouble();
        return robot_velocity(vx, va);
    } 
    return robot_velocity();
}

wheels_velocity inverse_kinematics(robot_velocity goal_vel) {
    double rpm_left = (goal_vel.vx * 30)/(PI*WHEEL_RADIUS) - (goal_vel.va * WHEEL_DISTANCE_FROM_CENTER) / (12*WHEEL_RADIUS);
    double rpm_right = (goal_vel.vx * 30)/(PI*WHEEL_RADIUS) + (goal_vel.va * WHEEL_DISTANCE_FROM_CENTER) / (12*WHEEL_RADIUS);
    return wheels_velocity(rpm_left, rpm_right);
}

position odometry(const position *old_position, double space_left, double space_right) {
    double space_x_cm = (space_left + space_right) / 2;
    double space_t_deg = (space_right - space_left) * 180 / (WHEEL_DISTANCE_FROM_CENTER * PI);
    double x = old_position.x + space_x_cm * cos((old_position.theta + space_t_deg/2)*PI/180);
    double y = old_position.y + space_x_cm * sin((old_position.theta + space_t_deg/2)*PI/180);
    double theta = old_position.theta + space_t_deg;
    return position(x, y, theta);
}

robot_velocity forward_kinematics(wheels_velocity vel) {
    double left_cms = vel.left * 2*PI*WHEEL_RADIUS / 60;
    double right_cms = vel.right * 2*PI*WHEEL_RADIUS / 60;

    double vx_cms = (left_cms + right_cms) / 2;
    double va_degs = (right_cms - left_cms) * 180 / (WHEEL_DISTANCE_FROM_CENTER * PI);

    return robot_velocity(vx_cms, va_degs);
}

String produce_response(position pos, robot_velocity vel, wheels_velocity w_vel, unsigned long loop_time) {
    String s_vel = "VEL " + String(vel.vx) + " " + String(vel.va) + ";";
    String s_pos = "POS " + String(pos.x) + " " + String(pos.y) + " " + String(pos.theta) + ";";
    String s_wvel = "WVEL " + String(w_vel.left) + " " + String(w_vel.right) + ";";
    String s_time = "TIME " + String(loop_time) + ";";
    String data_res = s_pos + s_vel + s_wvel + s_time;
    return data_res;
}


// void idle_loop() {
//     if (Serial.available() > 0) {
//         read_serial();
//     } else delay(100);
    
//     return;
// }

// void running_loop() {
//     unsigned long t_start = millis();
//     received_serial = false;

//     if (Serial.available() > 0) {
//         String s = Serial.read("\n");
//         if (s == "STOP\n") {
//             Serial.println("OK");
//             controller_mode = IDLE;
//             return;
//         }
//         received_serial = true;

//         goal_velocity = read_data(s);
//         goal_w_vel = inverse_kinematics(goal_velocity);
//     }

//     motor_left.update(goal_w_vel.left);
//     motor_right.update(goal_w_vel.right);

//     if (received_serial) {
//         int d_ticks_l = motor_left.ticks() - prev_ticks_l;
//         int d_ticks_r = motor_right.ticks() - prev_ticks_r;
//         prev_ticks_l += d_ticks_l;
//         prev_ticks_r += d_ticks_r;

//         String res = produce_response(motor_left, motor_right, d_ticks_l, d_ticks_r, dist, millis() - t_start);
//         Serial.println(res);
//     }

//     unsigned long dt = millis() - t_start;
//     if (dt < CONTROLLER_UPDATE_TIME)
//         delay(CONTROLLER_UPDATE_TIME - dt);



    // if (serial_loop) {
    //     Serial.println("HI!");
    //     if (Serial.available() > 0) {
    //         String s = Serial.readStringUntil("\n");
    //         if (s == "STOP\n") {
    //             controller_mode = IDLE;
    //             return;
    //         }
    //         // Serial.print("Received: ");
    //         // Serial.println(s);

    //         goal_velocity = read_data(s);
    //         goal_w_vel = inverse_kinematics(goal_velocity);
    //     }
    //     double actual_rpm_left = motor_left.update(goal_w_vel.left);
    //     double actual_rpm_right = motor_right.update(goal_w_vel.right);
        
    //     int d_ticks_l = motor_left.ticks() - prev_ticks_l;
    //     int d_ticks_r = motor_right.ticks() - prev_ticks_r;
    //     prev_ticks_l += d_ticks_l;
    //     prev_ticks_r += d_ticks_r;

    //     // String res = produce_response(d_ticks_l, d_ticks_r, dist, motor_left, motor_right);
    //     // String debug = produce_debugger(goal_w_vel, actual_rpm_left, actual_rpm_right, motor_left.ticks(), motor_right.ticks());
    //     // Serial.println(res);

    // } else {
    //     motor_left.update(goal_w_vel.left);
    //     motor_right.update(goal_w_vel.right);
    // }

    // unsigned long dt = millis() - t_start;
    // if (dt < CONTROLLER_UPDATE_TIME)
    //     delay(CONTROLLER_UPDATE_TIME - dt);
// }







// String produce_debugger(wheels_vel goal, double actual_l, double actual_r, int ticks_l, int ticks_r) {
//     String s = String(goal.left) +  " " + String(actual_l) + " " + String(goal.right) + " " + String(actual_r);
//     s += "\n " +  String(ticks_l) + " " + String(ticks_r);
//     return s;
// }