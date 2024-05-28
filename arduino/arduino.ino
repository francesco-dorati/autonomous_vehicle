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

#define COUNTS_PER_REV 1500
#define WHEEL_RADIUS 3.4 // cm
#define WHEEL_DISTANCE_FROM_CENTER 12 // cm
#define CONTROLLER_FREQ 100 // Hz
#define CONTROLLER_UPDATE_TIME (1000/CONTROLLER_FREQ)
#define SERIAL_RATE 50  // Hz
#define SERIAL_UPDATE_TIME (1000/SERIAL_RATE)

/*
TODO
- [arduino] create idle state, handshake with serial frequency
- [rpi] stop running of a server if the other control is on
- [console] add settings with all constants


*/
#include "Arduino.h"
#include "MotorController.h"


enum mode {
    IDLE = 0,
    RUNNING = 1
} ;

struct state_vel {
    double vx;    // cm/s
    double va;  // degrees/s
    state_vel(): vx(0), va(0) {};
    state_vel(float x, float t): vx(x), va(t) {};
};

struct wheels_vel {
    double left; // rpm
    double right; // rpm
    wheels_vel(): left(0), right(0) {};
    wheels_vel(float l, float r): left(l), right(r) {};
};

MotorController motor_left(PWM_ML, IN1_ML, IN2_ML, ENCA_ML, ENCB_ML, COUNTS_PER_REV, true);
MotorController motor_right(PWM_MR, IN1_MR, IN2_MR, ENCA_MR, ENCB_MR, COUNTS_PER_REV, false);

mode controller_mode = IDLE;

state_vel goal_velocity;
wheels_vel goal_w_vel;
bool received_serial = false;

long prev_ticks_l = 0;
long prev_ticks_r = 0;

float dist[4];

void setup() {
    Serial.begin(115200);
}

void loop() {
    if (controller_mode == IDLE) {
        idle_loop();
    } else {
        running_loop();
    }
}
 
void idle_loop() {
    if (Serial.available() > 0) {
        String s = Serial.readStringUntil("\n");
        
        if (s == "START\n"){
            controller_mode = RUNNING;
            Serial.println("OK");
        }
        
    } else delay(100);
    
    return;
}

void running_loop() {
    unsigned long t_start = millis();
    received_serial = false;

    if (Serial.available() > 0) {
        String s = Serial.readStringUntil("\n");
        if (s == "STOP\n") {
            Serial.println("OK");
            controller_mode = IDLE;
            return;
        }
        received_serial = true;

        goal_velocity = read_data(s);
        goal_w_vel = inverse_kinematics(goal_velocity);
    }

    motor_left.update(goal_w_vel.left);
    motor_right.update(goal_w_vel.right);

    if (received_serial) {
        int d_ticks_l = motor_left.ticks() - prev_ticks_l;
        int d_ticks_r = motor_right.ticks() - prev_ticks_r;
        prev_ticks_l += d_ticks_l;
        prev_ticks_r += d_ticks_r;

        String res = produce_response(motor_left, motor_right, d_ticks_l, d_ticks_r, dist);
        Serial.println(res);
    }

    unsigned long dt = millis() - t_start;
    if (dt < CONTROLLER_UPDATE_TIME)
        delay(CONTROLLER_UPDATE_TIME - dt);



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
}

state_vel read_data(String s) {
    state_vel vel;  
    int spaceIndex = s.indexOf(' ');

    vel.vx = s.substring(0, spaceIndex).toDouble();
    vel.va = s.substring(spaceIndex + 1).toDouble();

    return vel;
}

wheels_vel inverse_kinematics(state_vel goal_vel) {
    wheels_vel w;
    w.left = (goal_vel.vx * 60) / (2*PI*WHEEL_RADIUS) - (goal_vel.va * WHEEL_DISTANCE_FROM_CENTER) / (6*WHEEL_RADIUS);
    w.right = (goal_vel.vx * 60) / (2*PI*WHEEL_RADIUS) + (goal_vel.va * WHEEL_DISTANCE_FROM_CENTER) / (6*WHEEL_RADIUS);

    return w;
}


String produce_response(MotorController ml, MotorController mr, int ticks_l, int ticks_r, float dist[4]) {
    String left = "LEFT " + String(ml.goal_rpm) + " " + String(ml.actual_rpm) + " " + String(ml.power) + " " + String(ticks_l) + "; ";
    String right = "RIGHT " + String(mr.goal_rpm) + " " + String(mr.actual_rpm) + " " + String(mr.power) + " " + String(ticks_r) + "; ";

    String data_res = left + right;
    return data_res;
}

// String produce_debugger(wheels_vel goal, double actual_l, double actual_r, int ticks_l, int ticks_r) {
//     String s = String(goal.left) +  " " + String(actual_l) + " " + String(goal.right) + " " + String(actual_r);
//     s += "\n " +  String(ticks_l) + " " + String(ticks_r);
//     return s;
// }