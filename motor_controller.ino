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
#define WHEEL_DISTANCE 24 // cm
#define CONTROLLER_FREQ 50 // Hz


#include "Arduino.h"
#include <string.h>
#include "MotorController.h"

struct state {
    int x;
    int y;
    int theta;
    state(): x(0), y(0), theta(0) {};
    state(int x, int y, int t): x(x), y(y), theta(t) {};
};

MotorController motor_left(PWM_ML, IN1_ML, IN2_ML, ENCA_ML, ENCB_ML, COUNTS_PER_REV, true);
MotorController motor_right(PWM_MR, IN1_MR, IN2_MR, ENCA_MR, ENCB_MR, COUNTS_PER_REV, false);
void setup() {
  Serial.begin(9600);
}

int goal_rpm_left = 0; // -125 125
int goal_rpm_right = 0; // -125 125
void loop() {
  if (Serial.available() > 0) {
    String s = Serial.readStringUntil("\n");
    sscanf(s.c_str(), "%d %d", &goal_rpm_left, &goal_rpm_right);
    Serial.println(goal_rpm_right);
  }


  // set motor power
  float actual_rpm_left = motor_left.velocity(goal_rpm_left);
  float actual_rpm_right = motor_right.velocity(goal_rpm_right);
  Serial.print(goal_rpm_left);
  Serial.print(" ");
  Serial.print(actual_rpm_left);
  Serial.print(" ");
  Serial.print(goal_rpm_right);
  Serial.print(" ");
  Serial.println(actual_rpm_right);

  delay(1000/CONTROLLER_FREQ);
}

state BaseController::update_state(float rpm_right, float rpm_left) {
    float speed_right, speed_left;
    float robot_linear_velocity, robot_angular_velocity;
    unsigned long delta_t;

    if (_odometry_prev_time == 0) {
        _odometry_prev_time = millis();
        return _state;
    }

    // calculate delta time
    delta_t = millis() - _odometry_prev_time;

    // calculate speeds
    speed_left = rps_left * _wheel_radius;
    speed_right = rps_right * _wheel_radius;
    
    // calculate robot velocities
    robot_linear_velocity = (speed_right + speed_left) / 2;
    robot_angular_velocity = (speed_right - speed_left) / _wheel_distance;
    state new_state = state(
        _state.x + robot_linear_velocity * cos(_state.theta) * delta_t,
        _state.y + robot_linear_velocity * sin(_state.theta) * delta_t,
        _state.theta + robot_angular_velocity * delta_t
    );

    return new_state;
}
