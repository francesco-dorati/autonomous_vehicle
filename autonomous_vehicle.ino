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
#define CONTROLLER_FREQ 50 // Hz


#include "Arduino.h"
#include <string.h>
#include "MotorController.h"


MotorController motor_left(PWM_ML, IN1_ML, IN2_ML, ENCA_ML, ENCB_ML, COUNTS_PER_REV, true);
MotorController motor_right(PWM_MR, IN1_MR, IN2_MR, ENCA_MR, ENCB_MR, COUNTS_PER_REV, false);

enum Mode {
    IDLE,
    AUTO,
    MANUAL
};


struct state {
    float x;    // cm
    float y;    // cm
    float theta;  // degrees
    state(): x(0), y(0), theta(0) {};
    state(float x, float y, float t): x(x), y(y), theta(t) {};
};

struct state_vel {
    float x_velocity;    // cm/s
    float y_velocity;    // cm/s
    float theta_velocity;  // degrees/s
    state_vel(): x_velocity(0), y_velocity(0), theta_velocity(0) {};
    state_vel(float x, float y, float t): x_velocity(x), y_velocity(y), theta_velocity(t) {};
};

struct wheels_vel {
    float left; // rpm
    float right; // rpm
    wheels_vel(): left(0), right(0) {};
    wheels_vel(float l, float r): left(l), right(r) {};
};

Mode mode = IDLE;
void setup() {
    Serial.begin(9600);
}


void loop() {
    // Serial.println(mode);
    if (mode == IDLE) {
        Serial.println("IDLE");
        while(!Serial.available()) delay(10); // wait for serial

        String s = Serial.readStringUntil('\n');
        if (s == "AUTO") mode = AUTO;
        else if (s == "MANUAL") mode = MANUAL;
    }

    if (mode == AUTO) {
        Serial.println("AUTO");
        while(!Serial.available()) delay(10); // wait for serial

        String s = Serial.readStringUntil('\n');
        if (s == "exit") {
            mode = IDLE;
            return;
        }

        char action;
        int dist;
        sscanf(s.c_str(), "%c %d", &action, &dist);

        if (action == 'm') {
            // while inside
        } else if (action == 'r') {
            // while inside
        }
        
    }

    if  (mode == MANUAL) {
        Serial.println("MANUAL");
        while(!Serial.available()) delay(5); // wait for serial
        state_vel goal_vel;

        String s = Serial.readStringUntil('\n');
        if (s == "exit") {
            mode = IDLE;
            return;
        }

        // read buffer
        for (unsigned int i = 0; i < s.length(); i++) {
            if (s[i] == 'f') 
                goal_vel.x_velocity += 15;
            else if (s[i] == 'b') 
                goal_vel.x_velocity -= 15;
            else if (s[i] == 'l')
                goal_vel.theta_velocity += 35;
            else if (s[i] == 'r') 
                goal_vel.theta_velocity -= 35;
        }

        // give motor velocity
        wheels_vel goal_wheels = inverse_kinematics(goal_vel);
        motor_left.set_velocity(goal_wheels.left);
        motor_right.set_velocity(goal_wheels.right);
        
    }
}

 wheels_vel inverse_kinematics(state_vel goal_vel) {
    float x = goal_vel.x_velocity;
    float theta = goal_vel.theta_velocity;

    float left = (x*60 - theta*WHEEL_DISTANCE_FROM_CENTER/6) / WHEEL_RADIUS;
    float right = (x*60 + theta*WHEEL_DISTANCE_FROM_CENTER/6) / WHEEL_RADIUS;

    return wheels_vel(left, right);
 }


