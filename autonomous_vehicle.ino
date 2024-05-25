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
#include <string.h>
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

unsigned long previous_millis_controller = 0; 
unsigned long previous_millis_serial = 0; 

void setup() {
    Serial.begin(115200);
    Serial.println("OK");
}


state_vel goal_velocity;
bool serial_ok = true;
void loop() {
    unsigned long t_start = millis()

    if (serial_ok && Serial.available() > 0) {
        String s = Serial.readStringUntil("\n");
        float vx, va;
        sscanf(s.c_str(), "%f %f", &vx, &va);
        goal_velocity = state_vel(vx, va);
    }
    serial_ok = !serial_ok;

    wheels_vel w_vel = inverse_kinematics(goal_velocity);
    motor_left.set_velocity(w_vel.left)
    motor_right.set_velocity(w_vel.right)

    // update change in position
    



    unsigned long current_millis = millis();

    if (current_millis - previous_millis_controller >= CONTROLLER_UPDATE_TIME) {
        previous_millis_controller = current_millis;
        
        // PID control
        motor_left.

        // control
    }

    if (current_millis - previous_millis_serial >= SERIAL_UPDATE_TIME) {
        previous_millis_serial = current_millis;

        // send previous state
        
        // update goal velocity
        if (Serial.available() > 0) {
            String s = Serial.readStringUntil("\n");
            float vx, va;
            sscanf(s.c_str(), "%f %f", &vx, &va);
            goal_velocity = state_vel(vx, va);
        }

    }


    // Serial.println(mode);
    if (Serial.available() > 0) {
        state goal(x, y, t);
        state_vel goal_vel(x, y, t);
        wheels_vel wheels = inverse_kinematics(goal_vel);
        motor_left.velocity(wheels.left);
        motor_right.velocity(wheels.right);
    }

}

 wheels_vel inverse_kinematics(state_vel goal_vel) {
    float x = goal_vel.x_velocity;
    float theta = goal_vel.theta_velocity;

    float left = (x*60 - theta*WHEEL_DISTANCE_FROM_CENTER/6) / WHEEL_RADIUS;
    float right = (x*60 + theta*WHEEL_DISTANCE_FROM_CENTER/6) / WHEEL_RADIUS;

    return wheels_vel(left, right);
 }


