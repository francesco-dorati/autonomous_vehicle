// #define PWM_MR 5     // BROWN (D5)
// #define IN1_MR 9     // RED (D9)
// #define IN2_MR 4     // ORANGE (D4)
// #define ENCA_MR 3    // GREEN (D3)
// #define ENCB_MR 19   // YELLOW (A5)

// #define PWM_ML 6     // BLUE (D6)
// #define IN1_ML 7     // YELLOW (D7)
// #define IN2_ML 8     // GREEN (D8)
// #define ENCA_ML 2    // GREEN (D2)
// #define ENCB_ML 18   // YELLOW (A4)

// #define COUNTS_PER_REV 367
// #define WHEEL_RADIUS 3.4 // cm
// #define WHEEL_DISTANCE 24 // cm

// #define KP 0.5
// #define KI 1.2

// // #define SENSOR_1_TRIG 15
// // #define SENSOR_1_ECHO 14

// // #define SENSOR_2_TRIG 17
// // #define SENSOR_2_ECHO 16


// // #define LOOP_DELAY 100

// // #include "BaseController.h"
// #include "Arduino.h"
// // #include "MotorController.h"
// #include <string.h>

// /*
// CAVI MOTORI
// ROSSO - 12V motore
// BIANCO - GND MOTORE

// GIALLO - ENC
// VERDE - ENC
// BLU - 5V ENC
// NERO - GND ENC

// MAX SPEED RPM 170 (130 on load)
// */

// void setup() {
//   Serial.begin(9600);

//   // MOTOR 1
//   pinMode(PWM_ML, OUTPUT);
//   pinMode(IN1_ML, OUTPUT);
//   pinMode(IN2_ML, OUTPUT);
//   pinMode(ENCA_ML, INPUT);
//   pinMode(ENCB_ML, INPUT);

//   // MOTOR 2
//   pinMode(PWM_MR, OUTPUT);
//   pinMode(IN1_MR, OUTPUT);
//   pinMode(IN2_MR, OUTPUT);
//   pinMode(ENCA_MR, INPUT);
//   pinMode(ENCB_MR, INPUT);

//   attachInterrupt(digitalPinToInterrupt(ENCA_ML), read_encoder_left, RISING);
//   attachInterrupt(digitalPinToInterrupt(ENCA_MR), read_encoder_right, RISING);
// }

// int goal_cms_robot = 0, goal_rpm_robot = 0; // cm/s, rpm
// float goal_rpm_left = 0, goal_rpm_right = 0;
// unsigned long encoder_prev_time = 0;
// int encoder_counts_left = 0, encoder_counts_right = 0;
// int encoder_prev_counts_left = 0, encoder_prev_counts_right = 0;
// float error_integral_left = 0, error_integral_right = 0;

// void loop() {
//   if (Serial.available() > 0) {
//     read_serial();
//     // Serial.print("LIN: "); Serial.print(goal_cms_robot); Serial.print(" ANG: "); Serial.println(goal_rpm_robot);

//     // update goal velocities
//     goal_rpm_left = (60*goal_cms_robot/WHEEL_RADIUS) + (WHEEL_DISTANCE*goal_rpm_robot)/(2*WHEEL_RADIUS);
//     goal_rpm_right = (60*goal_cms_robot/WHEEL_RADIUS) - (WHEEL_DISTANCE*goal_rpm_robot)/(2*WHEEL_RADIUS);;
    
//     // Serial.print("LEFT_RPM: "); Serial.print(goal_rpm_left);Serial.print(" RIGHT_RPM: "); Serial.println(goal_rpm_right); Serial.println();
//   }


//   // update motor velocities
//   if (encoder_prev_time == 0) {
//     encoder_prev_time = millis();
//     Serial.println(" SKIP");
//     return;
//   }

//   int delta_counts_left = encoder_counts_left - encoder_prev_counts_left;
//   encoder_prev_counts_left = encoder_counts_left;

//   int delta_counts_right = encoder_counts_right - encoder_prev_counts_right;
//   encoder_prev_counts_right = encoder_counts_right;

//   float delta_t_ms = (float) (millis() - encoder_prev_time);
//   encoder_prev_time = millis();

//   float actual_rpm_left = (delta_counts_left*60000.0) / (delta_t_ms*COUNTS_PER_REV); 
//   float actual_rpm_right = (delta_counts_right*60000.0) / (delta_t_ms*COUNTS_PER_REV);

//   // calculate the error
//   float error_rpm_left = goal_rpm_left - actual_rpm_left;
//   float error_rpm_right = goal_rpm_right - actual_rpm_right;

//   // PID 
//   error_integral_left =  error_integral_left + error_rpm_left*(delta_t_ms/1000);
//   float power_left = KP*error_rpm_left + KI*error_integral_left;


//   error_integral_right = error_integral_right + error_rpm_right*(delta_t_ms/1000);
//   float power_right = KP*error_rpm_right + KI*error_integral_right;

//   Serial.print(goal_rpm_right);
//   Serial.print(" ");
//   Serial.print(actual_rpm_right);
//   Serial.print(" ");
//   Serial.print(KP*error_rpm_right);
//   Serial.print(" ");
//   Serial.print(KI*error_integral_right);
//   Serial.print(" ");
//   Serial.println(power_left);

//   // saturation
//   if (power_left > 255) power_left = 255;
//   else if (power_left < -255) power_left = -255;

//   if (power_right > 255) power_right = 255;
//   else if (power_right < -255) power_right = -255;

//   if (error_rpm_left == 0) power_left = 0;
//   if (error_rpm_right == 0) power_right = 0;


//   // set motor power
//   setPowerLeft(power_left);
//   setPowerRight(power_right);
// }

// void read_serial() {
//   String input_string = Serial.readStringUntil("\n");
//   sscanf(input_string.c_str(), "%d %d", &goal_cms_robot, &goal_rpm_robot);
// }

// void read_encoder_left() {
//   // Serial.println("ENC_B");
//   if (digitalRead(ENCB_ML) > 0) {
//     encoder_counts_left--;
//   } else {
//     encoder_counts_left++;
//   }
// }

// void read_encoder_right() {
//   if (digitalRead(ENCB_MR) > 0) {
//     encoder_counts_right++;
//   } else {
//     encoder_counts_right--;
//   }
// }

// void setPowerLeft(int power) {
//   if (power > 0) {
//     digitalWrite(IN1_ML, LOW);
//     digitalWrite(IN2_ML, HIGH);
//     analogWrite(PWM_ML, power);
//   } else if (power < 0) {
//     digitalWrite(IN1_ML, HIGH);
//     digitalWrite(IN2_ML, LOW);
//     analogWrite(PWM_ML, -power);
//   } else {
//     digitalWrite(IN1_ML, LOW);
//     digitalWrite(IN2_ML, LOW);
//     analogWrite(PWM_ML, 0);
//   }
// }

// void setPowerRight(int power) {
//   if (power > 0) {
//     digitalWrite(IN1_MR, LOW);
//     digitalWrite(IN2_MR, HIGH);
//     analogWrite(PWM_MR, power);
//   } else if (power < 0) {
//     digitalWrite(IN1_MR, HIGH);
//     digitalWrite(IN2_MR, LOW);
//     analogWrite(PWM_MR, -power);
//   } else {
//     digitalWrite(IN1_MR, LOW);
//     digitalWrite(IN2_MR, LOW);
//     analogWrite(PWM_MR, 0);
//   }
// }
