#define MOTOR_1_ENCA 3    // YELLOW (A5)
#define MOTOR_1_ENCB 19    // GREEN (D3)
#define MOTOR_1_PWM 5     // BROWN (D5)
#define MOTOR_1_IN1 2     // RED (D2)
#define MOTOR_1_IN2 4     // ORANGE (D4)

#define MOTOR_2_ENCA 9    // YELLOW (A4)
#define MOTOR_2_ENCB 18     // GREEN (D9)
#define MOTOR_2_PWM 6     // BLUE (D6)
#define MOTOR_2_IN1 7     // YELLOW (D7)
#define MOTOR_2_IN2 8     // GREEN (D8)


#define SENSOR_1_TRIG 15
#define SENSOR_1_ECHO 14

#define SENSOR_2_TRIG 17
#define SENSOR_2_ECHO 16

#define COUNTS_PER_REV 35
#define WHEEL_RADIUS 3.4 // cm
#define WHEEL_DISTANCE 24 // cm

// #define LOOP_DELAY 100

#include "BaseController.h"

/*
CAVI MOTORI
ROSSO - 12V motore
BIANCO - GND MOTORE

GIALLO - ENC
VERDE - ENC
BLU - 5V ENC
NERO - GND ENC

MAX SPEED RPM 170 (130 on load)
*/

BaseController controller(
  MOTOR_1_PWM, MOTOR_1_IN1, MOTOR_1_IN2, MOTOR_1_ENCA, MOTOR_1_ENCB,
  MOTOR_2_PWM, MOTOR_2_IN1, MOTOR_2_IN2, MOTOR_2_ENCA, MOTOR_2_ENCB,
  SENSOR_1_TRIG, SENSOR_1_ECHO,
  WHEEL_RADIUS, WHEEL_DISTANCE, COUNTS_PER_REV
);

state robot_state;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  robot_state = state();
}

void loop() {
  state robot_state = controller.updateState(0, 10);
  Serial.print("X: " + robot_state.x + " Y: " + robot_state.y + " THETA: " + robot_state.theta + "\n");
  delay(100);
}

