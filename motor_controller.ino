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


#include "Arduino.h"
#include <string.h>
#include "MotorController.h"

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

  delay(20);
}
