#include <Wire.h>
#include <WiFiNINA.h>
#include <Arduino.h>

#define DEBUG 0

#define CONTROLLER_FREQ 50 // Hz
#define CONTROLLER_UPDATE_TIME_MS (1000/CONTROLLER_FREQ)

#define FL_trig 2
#define FL_echo 3
#define FR_trig 4
#define FR_echo 5
#define RL_trig 6
#define RL_echo 7
#define RR_trig 8
#define RR_echo 9
#define LA_encoder 10
#define LB_encoder 11
#define RA_encoder A1
#define RB_encoder A2
#define battery_reader A0

#define wheel_radius_mm 34 // mm
#define wheel_distance_mm 240 // mm
#define counts_per_rev 370
#define half_sound_speed_mm_us 0.1715 // mm/us

const long wheel_circumference_mm = 2*PI*wheel_radius_mm;

short battery_voltage_mv = 0; // millivolts
short dist_mm[4] = {0, 0, 0, 0}; // FL FR RL RR
short wheel_velocities[2] = {0, 0}; // vl, vr (mm/s)
short robot_velocities[2] = {0, 0}; // vx (mm/s), vt (mrad/s)
long robot_position[3] = {0, 0, 0}; // x, y (um), theta (urad)

bool battery_reader_running = true;
bool sensors_running = false;
bool encoders_running = false;

// sensor
int sensor_n = 0; // fl, rl, fr, rr
// encoder
volatile long ticks_l, ticks_r;
long prev_ticks_l, prev_ticks_r, prev_time_encoder_us;

void left_tick();
void right_tick();
void update_encoders();

void update_battery();
void update_sensors();
int get_distance_mm(uint8_t trig, uint8_t echo);

void setup() {
    // serial
    Serial.begin(9600);
    Serial1.begin(115200);

    // sensors
    pinMode(FL_trig, OUTPUT);
    pinMode(FL_echo, INPUT);
    pinMode(FR_trig, OUTPUT);
    pinMode(FR_echo, INPUT);
    pinMode(RL_trig, OUTPUT);
    pinMode(RL_echo, INPUT);
    pinMode(RR_trig, OUTPUT);
    pinMode(RR_echo, INPUT);

    // encoders
    pinMode(LA_encoder, INPUT);
    pinMode(LB_encoder, INPUT);
    pinMode(RA_encoder, INPUT);
    pinMode(RB_encoder, INPUT);
    attachInterrupt(digitalPinToInterrupt(RA_encoder), right_tick, RISING);
    attachInterrupt(digitalPinToInterrupt(LA_encoder), left_tick, RISING);

    // battery reader
    pinMode(battery_reader, INPUT);
}

void loop() {
    unsigned long t_start = millis();
    // check serial
    if (Serial1.available() > 0) {
        char c = Serial1.read();
        
        Serial.print("received <");
        Serial.print(c);
        Serial.println(">. ");

        Serial1.print("ok");
    }

    // read battery
    long t = millis();
    if (battery_reader_running) update_battery();
    if (DEBUG) {
        Serial.print("Battery time: ");
        Serial.println(millis() - t);
    }
    
    // read sensors
    t = millis();
    if (sensors_running) update_sensors();
    if (DEBUG) {
        Serial.print("Sensors time: ");
        Serial.println(millis() - t);
    }

    // read encoders
    t = millis();
    if (encoders_running) update_encoders();
    if (DEBUG) {
        Serial.print("Encoders time: ");
        Serial.println(millis() - t);
    }
    
    long dt = millis() - t_start;
    if (DEBUG) {
        Serial.print("LOOP END, time: ");
        Serial.print(dt);
        Serial.println(" ms");
    }
    if (dt < CONTROLLER_UPDATE_TIME_MS)
        delay(CONTROLLER_UPDATE_TIME_MS - dt);
}

void requestEvent() {
    long t = millis();

    uint8_t data_buffer[32]; // Adjust size based on the data you are sending
    int index = 0;



    // // running data: 1 byte
    data_buffer[index++] = (battery_reader_running ? 0b100 : 0) |
                           (sensors_running ? 0b010 : 0) |
                           (encoders_running ? 0b001 : 0);
    
    // battery voltage: 2 bytes
    if (battery_reader_running) memcpy(&data_buffer[index], &battery_voltage_mv, sizeof(battery_voltage_mv));
    else memset(&data_buffer[index], 0, sizeof(battery_voltage_mv));
    index += sizeof(battery_voltage_mv);

    // sensor distance: 8 bytes
    if (sensors_running) memcpy(&data_buffer[index], dist_mm, sizeof(dist_mm));
    else memset(&data_buffer[index], 0, sizeof(dist_mm));
    index += sizeof(dist_mm);

    // wheel velocities: 4 bytes
    if (encoders_running) memcpy(&data_buffer[index], wheel_velocities, sizeof(wheel_velocities));
    else memset(&data_buffer[index], 0, sizeof(wheel_velocities));      
    index += sizeof(wheel_velocities);

    // robot velocities: 4 bytes
    if (encoders_running) memcpy(&data_buffer[index], robot_velocities, sizeof(robot_velocities));
    else memset(&data_buffer[index], 0, sizeof(robot_velocities));
    index += sizeof(robot_velocities);

    // robot position: 12 bytes
    int position[3];
    for (int i = 0; i < 3; i++) position[i] = (int)(robot_position[i]/1000);
    if (encoders_running) memcpy(&data_buffer[index], position, sizeof(position));
    else memset(&data_buffer[index], 0, sizeof(position));
    index += sizeof(position);

    data_buffer[index] = '\0'; // Null terminator

    // Send the entire packed data buffer
    // Wire.write(data_buffer, 32); // 32 FIXXX
    Wire.write(data_buffer, 1); // 32 FIXXX

    if (DEBUG) {
        Serial.print("requestEvent end, buffer sent, time: ");
        Serial.print(millis() - t);
        Serial.println(" ms");
    }

}
void receiveEvent(int bytes) {
    if (DEBUG) { Serial.println("receiveEvent start"); }

    while (bytes > 0) {
        char c = Wire.read();
        Serial.println(c);
        if (c == 'D' || c == 'd') {
            sensors_running = (c == 'D');
            dist_mm[0] = 0;
            dist_mm[1] = 0;
            dist_mm[2] = 0;
            dist_mm[3] = 0;
            
        } else if (c == 'E' || c == 'e') {
            encoders_running = (c == 'E');
            ticks_l = 0;
            ticks_r = 0;
            prev_ticks_l = 0;
            prev_ticks_r = 0;
            prev_time_encoder_us = micros();
            wheel_velocities[0] = 0;
            wheel_velocities[1] = 0;
            robot_velocities[0] = 0; 
            robot_velocities[1] = 0;
            robot_position[0] = 0; 
            robot_position[1] = 0;
            robot_position[2] = 0;
            
        } else if (c == 'B' || c == 'b') {
            battery_reader_running = (c == 'B');
            battery_voltage_mv = 0;
        } else {
            // Invalid command
        }
        bytes--;
    }
}

void left_tick() { 
    if (!encoders_running) return;
    if (digitalRead(LB_encoder) == HIGH) {
        ticks_l++;  // Forward
    } else {
        ticks_l--;  // Backward
    }
}

void right_tick() {
    if (!encoders_running) return;
    if (digitalRead(RB_encoder) == HIGH) {
        ticks_r++;  // Forward
    } else {
        ticks_r--;  // Backward
    }
} 

void update_encoders() {
    // delta ticks
    long d_ticks_l = ticks_l - prev_ticks_l;
    long d_ticks_r = ticks_r - prev_ticks_r;
    prev_ticks_l = ticks_l;
    prev_ticks_r = ticks_r;

    // delta time
    long t_us = micros();
    float d_time_us = (t_us - prev_time_encoder_us);
    prev_time_encoder_us = t_us;

    long dL_um = (1000 * d_ticks_l * wheel_circumference_mm) / counts_per_rev;
    long dR_um = (1000 * d_ticks_r * wheel_circumference_mm) / counts_per_rev;

    long dS_um = (dL_um + dR_um) / 2; // mm
    long dT_urad = (dR_um - dL_um) * 1000 / wheel_distance_mm; // urad

    // wheel velocities
    // mm/s = (ticks * mm/rev * us/s) / (ticks/rev * us)
    wheel_velocities[0] = (dL_um * 1000) / d_time_us; 
    wheel_velocities[1] = (dR_um * 1000) / d_time_us;

    // update state
    robot_velocities[0] /* mm/s */ =  (dS_um * 1000)/ d_time_us;
    robot_velocities[1] /* mrad/s */ = (dT_urad * 1000) / d_time_us; 

    robot_position[2] /* urad */ += dT_urad;
    robot_position[0] /* um */ += dS_um * cos(robot_position[2]/1000000.0);
    robot_position[1] /* um */ += dS_um * sin(robot_position[2]/1000000.0);

    if (DEBUG) {
        Serial.print("TICKS TOT ");
        Serial.println(ticks_r);
        Serial.print("Ticks: L ");
        Serial.print(d_ticks_l);
        Serial.print(", R");
        Serial.println(d_ticks_r);

        Serial.print("Time: ");
        Serial.print(d_time_us/1000);
        Serial.println("ms");

        Serial.print("dL ");
        Serial.print(dL_um);
        Serial.print(", dR ");
        Serial.println(dR_um);

        Serial.print(" dS ");
        Serial.print(dS_um);
        Serial.print(", dT ");
        Serial.println(dT_urad);

        Serial.print("Wheels (mm/s): L ");
        Serial.print(wheel_velocities[0]);
        Serial.print(", R");
        Serial.println(wheel_velocities[1]);

        Serial.print("Velocity: VX ");
        Serial.print(robot_velocities[0]);
        Serial.print(", VTHETA (mrad/s) ");
        Serial.println(robot_velocities[1]);

        Serial.print("Position (mm): X ");
        Serial.print(robot_position[0]/1000.0);
        Serial.print(", Y ");
        Serial.print(robot_position[1]/1000.0);
        Serial.print(", TH ");
        Serial.println(robot_position[2]/1000.0);
    }

}

void update_battery() {
    // Serial.println("BATTERY START");
    int pin_voltage_mv = (analogRead(battery_reader) / 1023.0) * 3300;
    battery_voltage_mv = pin_voltage_mv * 4;
}

void update_sensors() {
    // DIRECTION BASED
    if (encoders_running) {
        if (robot_velocities[0] > 0) {
            // front sensors
            if (sensor_n % 2 == 0) 
                dist_mm[0] = get_distance_mm(FL_trig, FL_echo);
            else 
                dist_mm[1] = get_distance_mm(FR_trig, FR_echo);
        } else if (robot_velocities[0] < 0) {
            // back sensors
            if (sensor_n % 2 == 0) 
                dist_mm[2] = get_distance_mm(RL_trig, RL_echo);
            else 
                dist_mm[3] = get_distance_mm(RR_trig, RR_echo);
        } else {
            // all sensors
            if (sensor_n == 0)
                dist_mm[0] = get_distance_mm(FL_trig, FL_echo);
            else if (sensor_n == 1)
                dist_mm[1] = get_distance_mm(FR_trig, FR_echo);
            else if (sensor_n == 2)
                dist_mm[2] = get_distance_mm(RL_trig, RL_echo);
            else if (sensor_n == 3) 
                dist_mm[3] = get_distance_mm(RR_trig, RR_echo);
        }
    }

    // NORMAL MODE
    // fl, rl, fr, rr 
    if (sensor_n == 0) // fl
        dist_mm[0] = get_distance_mm(FL_trig, FL_echo);
    else if (sensor_n == 1) // rl
        dist_mm[2] = get_distance_mm(RL_trig, RL_echo);
    else if (sensor_n == 2) // fr
        dist_mm[1] = get_distance_mm(FR_trig, FR_echo);
    else if (sensor_n == 3) // rr
        dist_mm[3] = get_distance_mm(RR_trig, RR_echo);

    sensor_n = (sensor_n + 1) % 4;
}

int get_distance_mm(uint8_t trig, uint8_t echo) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    // speed of sound = 343 mm/ms
    // half speed of sound = 171.5 mm/ms = 0.1715 mm/us
    long duration_us = pulseIn(echo, HIGH);
    return duration_us * half_sound_speed_mm_us;
}