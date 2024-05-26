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
#define WHEEL_DISTANCE_FROM_CENTER 24 // cm
#define CONTROLLER_FREQ 100 // Hz
#define CONTROLLER_UPDATE_TIME (1000/CONTROLLER_FREQ)
#define SERIAL_RATE 50  // Hz
#define SERIAL_UPDATE_TIME (1000/SERIAL_RATE)


#include "Arduino.h"
#include "MotorController.h"

struct state {
    float x;    // cm
    float y;    // cm
    float theta;  // degrees
    state(): x(0), y(0), theta(0) {};
    state(float x, float y, float t): x(x), y(y), theta(t) {};
};

struct state_vel {
    float vx;    // cm/s
    float va;  // degrees/s
    state_vel(): vx(0), va(0) {};
    state_vel(float x, float t): vx(x), va(t) {};
};

struct wheels_vel {
    float left; // rpm
    float right; // rpm
    wheels_vel(): left(0), right(0) {};
    wheels_vel(float l, float r): left(l), right(r) {};
};

MotorController motor_left(PWM_ML, IN1_ML, IN2_ML, ENCA_ML, ENCB_ML, COUNTS_PER_REV, true);
MotorController motor_right(PWM_MR, IN1_MR, IN2_MR, ENCA_MR, ENCB_MR, COUNTS_PER_REV, false);

void setup() {
    Serial.begin(115200);
    Serial.println("OK");
}

state_vel goal_velocity;
wheels_vel goal_w_vel;
bool serial_loop = false;

int d_ticks_l = 0;
int d_ticks_r = 0;
long prev_ticks_l = 0;
long prev_ticks_r = 0;

float dist[4];

void loop() {
    unsigned long t_start = millis()
    serial_loop = !serial_loop;

    if (serial_loop) {
        if (Serial.available() > 0) {
            String s = Serial.readStringUntil("\n");
            goal_velocity = read_data(s);
            goal_w_vel = inverse_kinematics(goal_velocity);
        }

        motor_left.set_velocity(w_vel.left);
        motor_right.set_velocity(w_vel.right);
        
        d_ticks_l = motor_left.ticks() - prev_ticks_l;
        d_ticks_r = motor_right.ticks() - prev_ticks_r;
        prev_ticks_l += d_ticks_l;
        prev_ticks_r += d_ticks_r;

        String res = produce_response(d_ticks_l, d_ticks_r, dist, motor_left, motor_right);
        Serial.println(r);

    } else {
        motor_left.set_velocity(w_vel.left);
        motor_right.set_velocity(w_vel.right);
    }

    unsigned long dt = millis() - t_start;
    if (dt < CONTROLLER_UPDATE_TIME)
        delay(CONTROLLER_UPDATE_TIME - dt);
    
}

state_vel read_data(String s) {
    float vx, va;
    sscanf(s.c_str(), "%f %f", &vx, &va);
    return state_vel(vx, va);
}

wheels_vel inverse_kinematics(state_vel goal_vel) {
    float x = goal_vel.x_velocity;
    float theta = goal_vel.theta_velocity;

    float left = (x*60 - theta*WHEEL_DISTANCE_FROM_CENTER/6) / WHEEL_RADIUS;
    float right = (x*60 + theta*WHEEL_DISTANCE_FROM_CENTER/6) / WHEEL_RADIUS;

    return wheels_vel(left, right);
}


String produce_response(int ticks_l, int ticks_r, float dist[4], MotorController ml, MotorController mr) {
    String data_res = "DATA TICKS " + String(ticks_l) + " " + String(ticks_r) + "; ";
    // String log_res = "LOG ";
    return data_res;
}