#include <Wire.h>
#include <WiFiNINA.h>

#define CONTROLLER_FREQ 100 // Hz
#define CONTROLLER_UPDATE_TIME_MS (1000/CONTROLLER_FREQ)

#define FL_trig 2
#define FL_echo 3
#define FR_trig 4
#define FR_echo 5
#define RL_trig 6
#define RL_echo 7
#define RR_trig 8
#define RR_echo 9
#define LA_encoder A0
#define LB_encoder A1
#define RA_encoder A2
#define RB_encoder A3
#define battery_reader A6

#define wheel_radius_mm 34 // mm
#define wheel_distance_mm 240 // mm
#define counts_per_rev 1495
#define half_sound_speed_mm_us 0.1715 // mm/us

const long wheel_circumference_mm = 2*PI*wheel_radius_mm;

short battery_voltage_mv = 0; // millivolts
short dist_mm[4] = {0, 0, 0, 0}; // FL FR RL RR
short wheel_velocities[2] = {0, 0}; // vl, vr (mm/s)
short robot_velocities[2] = {0, 0}; // vx (mm/s), vt (mrad/s)
int robot_position[3] = {0, 0, 0}; // x, y (mm), theta (mrad)

bool battery_reader_running = false;
bool sensors_running = false;
bool encoders_running = false;

// sensor
int side = 0; // left, right
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
    Wire.begin(0x08);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);
    
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
    attachInterrupt(digitalPinToInterrupt(LA_encoder), left_tick, RISING);
    attachInterrupt(digitalPinToInterrupt(RA_encoder), right_tick, RISING);

    // battery reader
    pinMode(battery_reader, INPUT);
}

void loop() {
    unsigned long t_start = millis();

    // read battery
    if (battery_reader_running) update_battery();
    
    // read sensors
    if (sensors_running) update_sensors();

    // read encoders
    if (encoders_running) update_encoders();
    
    long dt = millis() - t_start;
    if (dt < CONTROLLER_UPDATE_TIME_MS)
        delay(CONTROLLER_UPDATE_TIME_MS - dt);
}

void requestEvent() {
    uint8_t data_buffer[31]; // Adjust size based on the data you are sending
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
    if (encoders_running) memcpy(&data_buffer[index], robot_position, sizeof(robot_position));
    else memset(&data_buffer[index], 0, sizeof(robot_position));
    index += sizeof(robot_position);

    // Send the entire packed data buffer
    Wire.write(data_buffer, index);
}
void receiveEvent(int bytes) {
    while (bytes > 0) {
        char c = Wire.read();
        if (c == 'S' || c == 's') {
            sensors_running = (c == 'S');
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
    float d_time_s = (t_us - prev_time_encoder_us) / 1000000;
    prev_time_encoder_us = t_us;

    // wheel velocities
    // mm/s = (ticks * mm/rev * us/s) / (ticks/rev * us)
    wheel_velocities[0] = (d_ticks_l * wheel_circumference_mm) / (counts_per_rev * d_time_s); 
    wheel_velocities[1] = (d_ticks_r * wheel_circumference_mm) / (counts_per_rev * d_time_s);

    // update state
    robot_velocities[0] /* mm/s */ = (wheel_velocities[0] + wheel_velocities[1]) / 2; 
    robot_velocities[1] /* mrad/s */ = (wheel_velocities[1] - wheel_velocities[0]) * 1000 / wheel_distance_mm;
    robot_position[2] /* mrad */ += robot_velocities[1] * d_time_s;
    robot_position[0] /* mm */ += robot_velocities[0] * cos(robot_position[2]/1000) * d_time_s;
    robot_position[1] /* mm */ += robot_velocities[0] * sin(robot_position[2]/1000) * d_time_s;
}

void update_battery() {
    int pin_voltage_mv = (analogRead(battery_reader) / 4095.0) * 3300;
    battery_voltage_mv = pin_voltage_mv * 4;
}

void update_sensors() {
    if (side == 0) {
        // left
        dist_mm[0] = get_distance_mm(FL_trig, FL_echo);
        dist_mm[2] = get_distance_mm(RL_trig, RL_echo);
    } else if (side == 1) {
        // right
        dist_mm[1] = get_distance_mm(FR_trig, FR_echo);
        dist_mm[3] = get_distance_mm(RR_trig, RR_echo);
    }
    side = (side + 1) % 2;
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