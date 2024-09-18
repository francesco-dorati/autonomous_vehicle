#include <Wire.h>

#define CONTROLLER_FREQ 100 // Hz

#define FL_trig 2
#define FL_echo 3
#define FR_trig 4
#define FR_echo 5
#define BL_trig 6
#define BL_echo 7
#define BR_trig 8
#define BR_echo 9
#define LA_encoder A0
#define LB_encoder A1
#define RA_encoder A2
#define RB_encoder A3
#define battery_reader A6

#define wheel_radius 3.4 // cm
#define wheel_distance_from_center 24 // cm
#define counts_per_rev 1495

struct state {
    long vx;
    long vt;
    long x;
    long y;
    long theta;
    state(): vx(0), vt(0), x(0), y(0), theta(0) {};
    state(float vx, float vt, float x, float y, float theta): vx(vx), vt(vt), x(x), y(y), theta(theta) {};
}


bool sensors_running = false;
bool skip_loop = false; 
int side = 0; // left, right
float dist_cm[4] = {0, 0, 0, 0}; // FL FR BL BR

bool encoders_running = false;
long prev_ticks_l = 0, prev_ticks_r = 0;
volatile long ticks_l = 0, ticks_r = 0;
long prev_time_encoder = 0;
state encoders_state = state();

bool battery_reader_running = false;
float battery_voltage = 0.0;

void left_tick();
void right_tick();
void update_encoders();

void update_battery();
void update_sensors();
float trigger_sensors(uint8_t trig, uint8_t echo);

void setup() {
    Wire.begin(0x08);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);
    
    // sensors
    pinMode(FL_trig, OUTPUT);
    pinMode(FL_echo, INPUT);
    pinMode(FR_trig, OUTPUT);
    pinMode(FR_echo, INPUT);
    pinMode(BL_trig, OUTPUT);
    pinMode(BL_echo, INPUT);
    pinMode(BR_trig, OUTPUT);
    pinMode(BR_echo, INPUT);

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
    // read encoders
    if (encoders_running) {
        update_encoders();
    }

    // read battery
    if (battery_reader_running) {
        update_battery();
    }

    // read sensors
    if (sensors_running) {
        update_sensors();
    }


    unsigned long dt = millis() - t_start;
    if (dt < CONTROLLER_UPDATE_TIME)
        delay(CONTROLLER_UPDATE_TIME - dt);
}

void requestEvent() {
    // send data
}
void receiveEvent(int bytes) {
    // read data
}

void left_tick() { ticks_l++; }

void right_tick() { ticks_r++; }

void update_encoders() {
    // delta ticks
    long d_ticks_l = ticks_l - prev_ticks_l;
    long d_ticks_r = ticks_r - prev_ticks_r;
    prev_ticks_l = ticks_l;
    prev_ticks_r = ticks_r;

    // delta time
    long t = micros();
    long d_time_ms = (t - prev_time_encoder)/1000;
    prev_time_encoder = t;

    // wheel velocities
    long vl = (d_ticks_l * 2*PI*wheel_radius) / (counts_per_rev * d_time_ms);
    long vr = (d_ticks_r * 2*PI*wheel_radius) / (counts_per_rev * d_time_ms);

    // update state
    encoders_state.vx = (vl + vr) / 2;
    encoders_state.vt = (vr - vl) / wheel_distance;
    encoders_state.theta += encoders_state.vt * d_time_ms;
    encoders_state.x += encoders_state.vx * cos(encoders_state.theta) * d_time_ms;
    encoders_state.y += encoders_state.vx * sin(encoders_state.theta) * d_time_ms;
}


void update_battery() {
    float pin_voltage = (analogRead(battery_reader) / 4095.0) * 3.3;
    battery_voltage = pin_voltage * 4;
}

void update_sensors() {
    if (skip_loop) {
        skip_loop = !skip_loop;
        return;
    }

    if (side == 0) {
        // left
        dist_cm[0] = trigger_sensors(FL_trig, FL_echo);
        dist_cm[2] = trigger_sensors(BL_trig, BL_echo);
    } else if (side == 1) {
        // right
        dist_cm[1] = trigger_sensors(FR_trig, FR_echo);
        dist_cm[3] = trigger_sensors(BR_trig, BR_echo);
    }
}

float trigger_sensors(uint8_t trig, uint8_t echo) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    return pulseIn(echo, HIGH) / 58.2;
}