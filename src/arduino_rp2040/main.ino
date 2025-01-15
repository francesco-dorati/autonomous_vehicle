#include <Arduino.h>
#include <WiFiNINA.h>
#include <math.h>
#include <PID_v1.h>

/*    
    Motor Controller 
    Handles motor control, path following and encoders odometry calculations

    INPUTS (Serial1 115200) 
    ALL INPUTS ENDS WITH '\n'

    "PNG" -> "PNG"
        - ping

    "STP" 
        - stop motors

    "PWR <pow_l> <pow_r>" 
        - set target power
        - (pow_l, pow_r) in [-255, 255]

    "VEL <vx> <vt>" 
        - set target velocity
        - (vx, vt) in [mm/s, mrad/s]

    "PID <kp> <ki> <kd>"
        - change pid values

    "ORQ" -> "<x> <y> <theta>"
        - odometry request
        - (x, y) in [mm], theta in [mrad], 

    "ORS"
        - reset odometry values
    
    "OST <x> <y> <theta>"
        - set odometry values
        - (x, y) in [mm], theta in [mrad]

    "ODB" -> "<x> <y> <theta> <vx> <vy>"
        - get debug odometry values
        - (x, y) in [mm], theta in [mrad], 
        - (vl, va) in [mm/s, mrad/s]


    "PTH <n> <x1> <y1> <th1> ... <xn> <yn> <thn>"
        - set new path to follow
        - n: number of points
        - (x, y) in [mm], th in [mrad]
        - requirements:
            - max number of points simultaneously: 20
            - distance between targets: 100 mm < dist < 1000 mm
            - heading between targets < pi
    
    "APP <n> <x> <y> <th> ... <xn> <yn> <thn>"
        - append point to path
        - n: number of points
        - (x, y) in [mm], th in [mrad]

*/

#define BASE_INTERVAL 20 // ms
#define POSITION_CONTROL_INTERVAL 100 // ms
#define SERIAL_INTERVAL 100 // ms
#define DEBUG_INTERVAL 500 // ms

// STATE
enum State {
    STALL,
    POWER_CONTROL,
    VELOCITY_CONTROL,
    POSITION_CONTROL
};
State control_state = STALL;

// SERIAL
const char END_CHAR = '\n';
long last_serial_time = 0;
void handle_serial();

// GENERAL
class Position { // general unit
    public:
        int x, y, th;
        Position();
        Position(int x, int y, int th);
        void reset();
};
bool is_over(long last_time, int interval);
void sleep(long t_start);

// ENCODERS
#define LA_ENCODER_PIN 10 // LEFT YELLOW 
#define LB_ENCODER_PIN 11 // LEFT GREEN
#define RA_ENCODER_PIN 9  // RIGHT GREEN
#define RB_ENCODER_PIN 8  // RIGHT YELLOW
#define WHEEL_RADIUS_MM 34      // mm
#define WHEEL_CIRCUMFERENCE_UM 213628   // um  
#define WHEEL_DISTANCE_MM 240   // mm
#define COUNTS_PER_REV 370
#define TWO_PI_URAD 6283185     // 2π in microradians
#define PI_URAD 3141593.6       // π in microradians
#define TWO_PI_MRAD 6283.1853
#define PI_MRAD 3141.5936
volatile long ticks_l, ticks_r;
long prev_ticks_l, prev_ticks_r, prev_time_encoder_us;
int actual_wheel_velocities_m[2]; // vl, vr (mm/s)
int actual_robot_velocities_m[2]; // vx (mm/s), vt (mrad/s)
Position actual_robot_position_u = Position(); // x, y (um), theta (urad)
void left_tick();
void right_tick();
void reset_odometry();
void update_odometry();

// POWER CONTROL
#define MAX_POW 250
#define MIN_POW 0
#define ENA 4   // BROWN
#define IN1 2   // RED
#define IN2 3   // ORANGE
#define ENB 5   // BLUE
#define IN3 6   // YELLOW
#define IN4 7   // GREEN
int target_powers[2] = {0, 0}; // left, right
void send_motor_powers();
void stop_motors();

// // VELOCITY CONTROL
int target_robot_velocities_m[2] = {0, 0}; // vx (mm/s), vt (mrad/s)
int target_wheel_velocities_m[2] = {0, 0}; // vl, vr (mm/s)
double pid_goal_left = 0, pid_goal_right = 0; // PID goal
double pid_actual_left = 0, pid_actual_right = 0; // PID actual
double pid_output_left = 0, pid_output_right = 0; // PID output
double Kp = 0, Ki = 0, Kd = 0;
PID PID_LEFT(&pid_actual_left, &pid_output_left, &pid_goal_left, Kp, Ki, Kd, DIRECT);
PID PID_RIGHT(&pid_actual_right, &pid_output_right, &pid_goal_right, Kp, Ki, Kd, DIRECT);
void velocity_control();

// POSITION CONTROL
#define QUEUE_SIZE 20
class PositionQueue {
    public:
        PositionQueue();
        Position pop();
        void push(Position p);
        bool is_empty();
        void reset();
    private:
        Position queue[QUEUE_SIZE];
        int front, rear, count;
};
long last_position_control_time = 0;
Position target_robot_position_m = Position(); // x, y (mm), theta (mrad)
PositionQueue target_position_queue = PositionQueue();
void position_control();
void calculate_inverse_kinematic();
// dist
#define THRESH_DIST_MM 25   // mm   distance where target is reached
#define CHANGE_DIST_MM 50 // mm    distance where target is changed
#define MIN_LIN_SPEED_DIST_MM 100 // mm     distance where minimum speed is reached 
#define MIN_LIN_SPEED_MMS 0         // mm/s    minimum speed
#define MAX_LIN_SPEED_DIST_MM 500 // mm     distance where maximum speed is reached
#define MAX_LIN_SPEED_MMS 0       // mm/s    maximum speed
const double LIN_SPEED_SLOPE = (MAX_LIN_SPEED_MMS - MIN_LIN_SPEED_MMS)/(MAX_LIN_SPEED_DIST_MM - MIN_LIN_SPEED_DIST_MM); 
int distance_error_mm();
int distance_velocity(int dist_mm);
// heading
#define THRESH_HEAD_MRAD 100 // 5 degrees    angle where target is reached
#define START_HEAD_MRAD 600 
#define MIN_ANG_SPEED_HEAD_MRAD 1 // mrad/s  angle where minimum speed is reached
#define MIN_ANG_SPEED_MRADS 0 // mrad/s        minimum speed
#define MAX_ANG_SPEED_HEAD_MRAD 10 // mrad/s     angle where maximum speed is reached
#define MAX_ANG_SPEED_MRADS 100 // mrad/s      maximum speed
const int ANG_SPEED_SLOPE = (MAX_ANG_SPEED_MRADS - MIN_ANG_SPEED_MRADS)/(MAX_ANG_SPEED_HEAD_MRAD - MIN_ANG_SPEED_HEAD_MRAD);
int heading_error_mrad();
int heading_velocity(int heading_mrad);
// // alpha
#define THRESH_ALIGN_MRAD 200 // 10 degrees
#define VEL_ALIGN_MRADS 0 // 5 degrees
int align_error_mrad();
int align_velocity(int alpha_mrad);

// DEBUG
class Logger { // handle serial inside, check serial to be run every start of turn
    class SerialLogger {
        public:
            void received(char command[]);
            void ping();
            void stop_motors();
            void set_powers();
            void set_velocities();
            void set_pid();
            void request_odometry(int x_mm, int y_mm, int th_mm);
            void reset_odometry();
            void set_odometry();
            void complete_odometry(int x_mm, int y_mm, int th_mm, int v_lin, int v_ang);
            void set_path();
            void append_path();
            void invalid_command();

        private:
            bool on;
    };
    class OdometryLogger {
        public:
            void log(long ticks_l, long ticks_r, long d_ticks_l, long d_ticks_r, long d_time_us, 
                        long dL_um, long dR_um, long dS_um, long dT_urad);
        private:
            bool on;
            bool is_time;
    };
    class ControlLogger {
        public:
            void log_change_target_position(int dist_mm);
            void log_heading_adjustment(int dist_mm, int heading_error);
            void log_alignment_adjustment(int dist_mm, int align_error);
            void log_velocity();
            void log_power();
        private:
            bool on;
            bool is_time;
    };
    
    public:
        SerialLogger serial;
        OdometryLogger odometry;
        ControlLogger control;
        Logger(); // setup serial
        void check_connection();
        
    private:
        bool is_time;
        long last_check_time;

};
Debug debug_usb = Logger();



// SETUP
void setup() {
    Serial.begin(9600);
    Serial1.begin(115200);

    // Left Motor
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    // Right Motor
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Encoders
    pinMode(LA_ENCODER_PIN, INPUT);
    pinMode(LB_ENCODER_PIN, INPUT);
    pinMode(RA_ENCODER_PIN, INPUT);
    pinMode(RB_ENCODER_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(RA_ENCODER_PIN), right_tick, RISING);
    attachInterrupt(digitalPinToInterrupt(LA_ENCODER_PIN), left_tick, RISING);

    // PID
    PID_LEFT.SetMode(AUTOMATIC);
    PID_LEFT.SetOutputLimits(-MAX_POW, MAX_POW);
    PID_LEFT.SetSampleTime(BASE_INTERVAL);
    PID_RIGHT.SetMode(AUTOMATIC);
    PID_RIGHT.SetOutputLimits(-MAX_POW, MAX_POW);
    PID_RIGHT.SetSampleTime(BASE_INTERVAL);

    stop_motors();
    reset_odometry();
}


//// LOOP ////
void loop() {
    long t_start = millis();

    // handle serial commands
    if (is_over(last_serial_time, SERIAL_INTERVAL)) {
        // Serial1.write("HEllO!!\n");
        handle_serial();
        last_serial_time = millis();
    }

    // debug via usb serial
    /* log */ debug_usb.check_connection();

    // update odometry
    update_odometry();

    // stall
    if (control_state == STALL) {
        // stop motors???
        sleep(t_start);
        return;
    }

    // position control
    if (control_state == POSITION_CONTROL && is_over(last_position_control_time, POSITION_CONTROL_INTERVAL)) {
        // calculate target robot velocities
        position_control(); 
        calculate_inverse_kinematic(); // calculate target wheels velocity
        last_position_control_time = millis();
    }

    // velocity control
    if (control_state == POSITION_CONTROL || control_state == VELOCITY_CONTROL) {
        // compute motor powers
        velocity_control();
    }

    if (target_powers[0] == 0 && target_powers[1] == 0) { stop_motors(); }

    // send motor powers
    send_motor_powers();
    
    // delay
    sleep(t_start);
    
}

bool is_over(long last_time, int interval) {
    return (millis() - last_time) >= interval;
}
void sleep(long t_start) {
    int dt = millis() - t_start;
    if (dt < BASE_INTERVAL) 
        delay(BASE_INTERVAL - dt);
}

void stop_motors() {
    control_state = STALL;
    target_powers[0] = 0;
    target_powers[1] = 0;
    send_motor_powers();
    target_robot_velocities_m[0] = 0;
    target_robot_velocities_m[1] = 0;
    target_robot_position_m.reset();
    target_position_queue.reset();
}

long normalize_angle_urad(long angle) {
    return ((angle + (int) PI_URAD) % (int) TWO_PI_URAD) - PI_URAD;
}
long normalize_angle_mrad(long angle) {
    return ((angle + (int) PI_MRAD) % (int) TWO_PI_MRAD) - PI_MRAD;
}

Position::Position() {
    this->x = 0;
    this->y = 0;
    this->th = 0;
}
Position::Position(int x, int y, int th) {
    this->x = x;
    this->y = y;
    this->th = th;
}
void Position::reset() {
    this->x = 0;
    this->y = 0;
    this->th = 0;
}

// SERIAL
void handle_serial() {
    while (Serial1.available() >= 3) {
        char command[4];
        command[3] = '\0';

        Serial1.readBytes(command, 3);
        /* log */ debug_usb.serial.received(command); 

        if (strcmp(command, "PNG") == 0) {
            // ping
            Serial1.println("PNG");
            /* log */ debug_usb.serial.ping();

        } else if (strcmp(command, "STP") == 0) {
            // stop motors
            // "STP"
            stop_motors();
            /* log */ debug_usb.serial.stop_motors();

        } else if (strcmp(command, "PWR") == 0) {
            // power control
            // "PWR <pow_l> <pow_r>" 
            // if (Serial1.available() < 2) return;
            target_powers[0] = Serial1.parseInt();
            target_powers[1] = Serial1.parseInt();
            control_state = POWER_CONTROL;
            /* log */ debug_usb.serial.set_powers();

        } else if (strcmp(command, "VEL") == 0) {
            // velocity control
            // "VEL <vx> <vt>" 
            // if (Serial1.available() < 3) return;
            target_robot_velocities_m[0] = Serial1.parseInt();
            target_robot_velocities_m[1] = Serial1.parseInt();
            calculate_inverse_kinematic();
            control_state = VELOCITY_CONTROL;
            /* log */ debug_usb.serial.set_velocities();

        } else if (strcmp(command, "PID") == 0) {
            // pid values change
            // "PID <kp> <ki> <kd>"
            // if (Serial1.available() < 6) return;
            Kp = Serial1.parseFloat();
            Ki = Serial1.parseFloat();
            Kd = Serial1.parseFloat();
            PID_LEFT.SetTunings(Kp, Ki, Kd);
            PID_RIGHT.SetTunings(Kp, Ki, Kd);
            /* log */ debug_usb.serial.set_pid();

        } else if (strcmp(command, "ORQ") == 0) {
            // odometry request
            // "ORQ" -> "<x> <y> <theta>"
            int x_mm = actual_robot_position_u.x/1000;
            int y_mm = actual_robot_position_u.y/1000;
            int th_mm = actual_robot_position_u.th/1000;
            Serial1.print(x_mm); // x
            Serial1.print(" ");
            Serial1.print(y_mm); // y
            Serial1.print(" ");
            Serial1.println(th_mm); // theta
            /* log */ debug_usb.serial.request_odometry(x_mm, y_mm, th_mm);

        } else if (strcmp(command, "ORS") == 0) {
            // odometry reset
            // "ORS"
            reset_odometry();
            /* log */ debug_usb.serial.reset_odometry();

        } else if (strcmp(command, "OST") == 0) {
            // odometry set
            // "OST <x> <y> <theta>"
            int x_u = Serial1.parseInt()*1000; 
            int y_u = Serial1.parseInt()*1000; 
            int th_u = Serial1.parseInt()*1000; 
            actual_robot_position_u = Position(x_u, y_u, th_u);
            /* log */ debug_usb.serial.set_odometry();

        } else if (strcmp(command, "ODB") == 0) {
            // odometry debug
            // "ORQ" -> "<x> <y> <theta> <vl> <va>"
            int x_mm = actual_robot_position_u.x/1000;
            int y_mm = actual_robot_position_u.y/1000;
            int th_mm = actual_robot_position_u.th/1000;
            int v_lin = actual_robot_velocities_m[0];
            int v_ang = actual_robot_velocities_m[1];
            Serial1.print(x_mm); // x
            Serial1.print(" ");
            Serial1.print(y_mm); // y
            Serial1.print(" ");
            Serial1.print(th_mm); // theta
            Serial1.print(" ");
            Serial1.print(v_lin); // vx
            Serial1.print(" ");
            Serial1.println(v_ang); // vt
            /* log */ debug_usb.serial.complete_odometry(x_mm, y_mm, th_mm, v_lin, v_ang);

        } else if (strcmp(command, "PTH") == 0) {
            // set path to follow
            // "PTH <n> <x1> <y1> <th1> ... <xn> <yn> <thn>"
            target_position_queue.reset();
            int n = Serial1.parseInt();
            for (int i = 0; i < n; i++) {
                long x_mm = Serial1.parseInt();
                long y_mm = Serial1.parseInt();
                long th_mrad = Serial1.parseInt();
                target_position_queue.push(Position(x_mm, y_mm, th_mrad));
            }
            /* log */ debug_usb.serial.set_path();

        } else if (strcmp(command, "APP") == 0) {
            int n = Serial1.parseInt();
            for (int i = 0; i < n; i++) {
                long x_mm = Serial1.parseInt();
                long y_mm = Serial1.parseInt();
                long th_mrad = Serial1.parseInt();
                target_position_queue.push(Position(x_mm, y_mm, th_mrad));
            }
            /* log */ debug_usb.serial.append_path();

        } else {
            // invalid command
            // ignore
            /* log */ debug_usb.serial.invalid_command();
        }

        // read until END
        while (Serial1.available() > 0 && Serial1.read() != END_CHAR) {};
    }
}

// ODOMETRY
void left_tick() { 
    if (digitalRead(LB_ENCODER_PIN) == HIGH) ticks_l++;  // Forward
    else ticks_l--;  // Backward
}
void right_tick() {
    if (digitalRead(RB_ENCODER_PIN) == HIGH) ticks_r++;  // Forward
    else ticks_r--;  // Backward
} 
void reset_odometry() {
    ticks_l = 0;
    ticks_r = 0;
    prev_ticks_l = 0;
    prev_ticks_r = 0;
    prev_time_encoder_us = micros();
    actual_wheel_velocities_m[0] = 0;
    actual_wheel_velocities_m[1] = 0;
    actual_robot_velocities_m[0] = 0; 
    actual_robot_velocities_m[1] = 0;
    actual_robot_position_u.reset();
    target_robot_position_m.reset();
    target_position_queue.reset();
}
void update_odometry() {
    /*
        Updates the robot odometry
        reads: ticks_l, ticks_r
        writes: actual_wheel_velocities_m[2], actual_robot_velocities_m[2], actual_robot_position_u[3]
    */
    // delta ticks
    long d_ticks_l = ticks_l - prev_ticks_l;
    long d_ticks_r = ticks_r - prev_ticks_r;
    prev_ticks_l = ticks_l;
    prev_ticks_r = ticks_r;

    // delta time
    long t_us = micros();
    float d_time_us = (t_us - prev_time_encoder_us);
    prev_time_encoder_us = t_us;

    long dL_um = (d_ticks_l * WHEEL_CIRCUMFERENCE_UM) / COUNTS_PER_REV;
    long dR_um = (d_ticks_r * WHEEL_CIRCUMFERENCE_UM) / COUNTS_PER_REV;

    long dS_um = (dL_um + dR_um) / 2; // um
    long dT_urad = (dR_um - dL_um) * 1000 / WHEEL_DISTANCE_MM; // urad

    // wheel velocities
    // (um*1000)/us = nm/us = mm/s
    actual_wheel_velocities_m[0] = (dL_um * 1000) / d_time_us; 
    actual_wheel_velocities_m[1] = (dR_um * 1000) / d_time_us;

    // robot velocities
    // (um*1000)/us = nm/us = mm/s
    actual_robot_velocities_m[0] = (dS_um * 1000)/ d_time_us; 
    actual_robot_velocities_m[1] = (dT_urad * 1000) / d_time_us;
    

    // robot position
    // actual_robot_position_u.x += dS_um * cos(actual_robot_position_u.th/1000000.0); // um
    // actual_robot_position_u.y += dS_um * sin(actual_robot_position_u.th/1000000.0); // um
    actual_robot_position_u.x += dS_um * cos((actual_robot_position_u.th + dT_urad/2)/1000000.0); // um
    actual_robot_position_u.y += dS_um * sin((actual_robot_position_u.th + dT_urad/2)/1000000.0); // um
    actual_robot_position_u.th += dT_urad; // urad
    actual_robot_position_u.th = normalize_angle_urad(actual_robot_position_u.th);


    //DEBUG
    /* log */ debug_usb.odometry.log(ticks_l, ticks_r, d_ticks_l, d_ticks_r, d_time_us, 
                                                    dL_um, dR_um, dS_um, dT_urad);
}

// POWER CONTROL
void send_motor_powers() {
    /*  
        Sends the motor powers to the motors
        target_powers[0] -> left motor [-255, 255]
        target_powers[1] -> right motor [-255, 255]
    */

    // saturation
    // left
    if (abs(target_powers[0]) > MAX_POW) target_powers[0] = (target_powers[0] > 0) ? MAX_POW : -MAX_POW;
    if (abs(target_powers[0]) < MIN_POW) target_powers[0] = 0;
    // right
    if (abs(target_powers[1]) > MAX_POW) target_powers[1] = (target_powers[1] > 0) ? MAX_POW : -MAX_POW;
    if (abs(target_powers[1]) < MIN_POW) target_powers[1] = 0;

    // update motors
    // left
    if (target_powers[0] == 0){
        analogWrite(ENB, 0);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
    } else {
        analogWrite(ENB, abs(target_powers[0]));
        digitalWrite(IN3, (target_powers[0] > 0) ? LOW : HIGH);
        digitalWrite(IN4, (target_powers[0] > 0) ? HIGH : LOW);
    }
    // right
    if (target_powers[1] == 0){
        analogWrite(ENA, 0);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
    } else{
        analogWrite(ENA, abs(target_powers[1]));
        digitalWrite(IN1, (target_powers[1] > 0) ? LOW : HIGH);
        digitalWrite(IN2, (target_powers[1] > 0) ? HIGH : LOW);
    }

    /* log */ debug_usb.control.log_power();
}

// VELOCITY CONTROL
void velocity_control() {
    /*
        Computes the motor powers based on the target velocities
        (wheels velocities are already calculated)
        reads: target_wheel_velocities_m[2]
        writes: target_powers[2] 
    */
    // update pid variables 
    pid_actual_left = actual_wheel_velocities_m[0];
    pid_actual_right = actual_wheel_velocities_m[1];
    pid_goal_left = target_wheel_velocities_m[0];
    pid_goal_right = target_wheel_velocities_m[1];
    // compute PID
    PID_LEFT.Compute();
    PID_RIGHT.Compute();
    // update target powers
    target_powers[0] = (int) max(-255, min(pid_output_left, 255));
    target_powers[1] = (int) max(-255, min(pid_output_right, 255));

    /* log */ debug_usb.control.log_velocity();
}

// POSITION CONTROL
PositionQueue::PositionQueue() { 
    front = 0; rear = 0; count = 0;
}
Position PositionQueue::pop() {
    /*
        Returns the next point to follow
        requires: !is_empty()
    */
    if (this->is_empty()) return Position();
    Position p = queue[front];
    front = (front + 1) % QUEUE_SIZE;
    count--;
    return p;
}
void PositionQueue::push(Position p) {
    /*
        Adds a new point to the queue
        requires: count < QUEUE_SIZE
    */
    if (count == QUEUE_SIZE) return;
    queue[rear] = p;
    rear = (rear + 1) % QUEUE_SIZE;
    count++;
}
bool PositionQueue::is_empty() {
    return count == 0;
}
void PositionQueue::reset() {
    front = 0; rear = 0; count = 0;
}
void position_control() {
    /*
        Computes the target velocities based on the target position
        reads: target_robot_position_m[3], target_position_queue
        writes: target_robot_velocities_m[2]
        requires: distance between targets > CHANGE THRESH  
                  heading between targets: 0 < heading < pi
    */
    
    int dist_mm = distance_error_mm();// calculate distance
    while (dist_mm <= CHANGE_DIST_MM && !target_position_queue.is_empty()) { 
        // change target
        target_robot_position_m = target_position_queue.pop();
        /* log */ debug_usb.control.log_change_target_position(dist_mm);

        dist_mm = distance_error_mm();
    }

    if (dist_mm > THRESH_DIST_MM) { 
        // outside distance threashold
        // calculate heading error
        int heading_error = heading_error_mrad();
        if (heading_error > START_HEAD_MRAD) {
            // rotate
            target_robot_velocities_m[0] = 0;
            target_robot_velocities_m[1] = heading_velocity(heading_error);
        } else {
            // follow
            target_robot_velocities_m[0] = distance_velocity(dist_mm);
            target_robot_velocities_m[1] = heading_velocity(heading_error);
        }

        /* log */ debug_usb.control.log_heading_adjustment(dist_mm, heading_error);

    } else { 
        // inside distance threshold
        // check alignment
        int align_error = align_error_mrad();
        if (align_error < THRESH_ALIGN_MRAD) {
            // not aligned
            // rotate
            target_robot_velocities_m[0] = 0;
            target_robot_velocities_m[1] = align_velocity(align_error);
        } else {
            // aligned
            stop_motors();
        }
    
        /* log */ debug_usb.control.log_alignment_adjustment(dist_mm, align_error);
    }



}

int distance_error_mm() {
    /*
        Returns the distance error (mm) between target and actual position
        reads: target_robot_position_m, actual_robot_position_u
    */
   int dx_mm = target_robot_position_m.x - (actual_robot_position_u.x/1000);
   int dy_mm = target_robot_position_m.y - (actual_robot_position_u.y/1000);
   return sqrt(dx_mm*dx_mm + dy_mm*dy_mm);
}
int heading_error_mrad() {
    /*
        Returns the angle between target and actual position (mrad)
        reads: target_robot_position_m, actual_robot_position_u
    */
    int dx_mm = target_robot_position_m.x - (actual_robot_position_u.x/1000);
    int dy_mm = target_robot_position_m.y - (actual_robot_position_u.y/1000);
    return normalize_angle_mrad(atan2(dy_mm, dx_mm) * 1000);
    // return normalize_angle_mrad(target_robot_position_m[2] - actual_robot_position_u[2]/1000);
}
int align_error_mrad() {
    /*
        Returns the heading difference (mrad) between target and actual position
        reads: target_robot_position_m, actual_robot_position_u
    */
    return normalize_angle_mrad(target_robot_position_m.th - actual_robot_position_u.th/1000);
}

int distance_velocity(int dist_mm) {
    /*
        Returns the target velocity based on the distance error
        reads: d_mm
        returns:
            0                                           dist < TRESHOLD
            slope * (dist - MIN_DIST) + MIN_VEL         where MIN_DIST < dist < MAX_DIST
            staturated at MIN/MAX                       elsewhere
    */
    if (dist_mm < THRESH_DIST_MM) return 0;
    if (dist_mm < MIN_LIN_SPEED_DIST_MM) return MIN_LIN_SPEED_MMS;
    if (dist_mm > MAX_LIN_SPEED_DIST_MM) return MAX_LIN_SPEED_MMS;
    return (dist_mm - MIN_LIN_SPEED_DIST_MM) * LIN_SPEED_SLOPE + MIN_LIN_SPEED_MMS;
}
int heading_velocity(int heading_mrad) {
    /*
        Returns the target angular velocity based on the heading error
        reads: heading_mrad
        returns:
            0                                           heading < TRESHOLD
            slope * (heading - MIN_ANG) + MIN_VEL        where MIN_ANG < heading < MAX_ANG
            staturated at MIN/MAX                       elsewhere
    */
    if (abs(heading_mrad) < THRESH_HEAD_MRAD) return 0;
    if (abs(heading_mrad) < MIN_ANG_SPEED_HEAD_MRAD) return MIN_ANG_SPEED_MRADS;
    if (abs(heading_mrad) > MAX_ANG_SPEED_HEAD_MRAD) return MAX_ANG_SPEED_MRADS;
    return (heading_mrad - MIN_ANG_SPEED_HEAD_MRAD) * ANG_SPEED_SLOPE + MIN_ANG_SPEED_MRADS;
}
int align_velocity(int alpha_mrad) {
    /*
        Returns the target angular velocity based on the heading difference
        reads: alpha_mrad
        fixed speed cut of at THRESH_ALIGN_MRAD
    */
    return (abs(alpha_mrad) < THRESH_ALIGN_MRAD) ? 0 : VEL_ALIGN_MRADS; 
}
void calculate_inverse_kinematic() {
    /*
        Computes the target wheel velocities based on the target robot velocities
        reads: target_robot_velocities_m[2]
        writes: target_wheel_velocities_m[2]
    */
    target_wheel_velocities_m[0] = target_robot_velocities_m[0] - target_robot_velocities_m[1] * WHEEL_DISTANCE_MM / 2;
    target_wheel_velocities_m[1] = target_robot_velocities_m[0] + target_robot_velocities_m[1] * WHEEL_DISTANCE_MM / 2;
}

// DEBUG
Logger::Logger() {
    is_time = false;
    last_check_time = millis();
}
void Logger::check_connection() {
    if (is_over(last_check_time, DEBUG_INTERVAL)) {
        last_check_time = millis();
        if (!Serial) Serial.begin(9600);
        if (Serial) {
            is_time = true;
            odometry.is_time = true;
            control.is_time = true;

            Serial.println("ALIVE");
            Serial.print("CONTROL TYPE: ");
            switch (control_state) {
                case STALL: Serial.println("STALL"); break;
                case POWER_CONTROL: Serial.println("POWER CONTROL"); break;
                case VELOCITY_CONTROL: Serial.println("VELOCITY CONTROL"); break;
                case POSITION_CONTROL: Serial.println("POSITION CONTROL"); break;
                default: Serial.println("ERROR"); break;
            }
            

            if (Serial.available() > 0) {
                // check for debug instructions
                String command;
                command = Serial.readStringUntil('\n');
                if (command == "SER") {
                    serial.on = !serial.on;
                    Serial.println("SERIAL PRINTER " + (serial.on ? "ON" : "OFF"));
                } else (command == "ODO") {
                    odometry.on = !odometry.on;
                    Serial.println("ODOMETRY " + (odometry.on ? "ON" : "OFF"));
                } else (command == "CON") {
                    control.on = !control.on;
                    Serial.println("CONTROL " + (control.on ? "ON" : "OFF"));
                }
            }
        }

    } else {
        is_time = false;
        odometry.is_time = false;
        control.is_time = false;
    }
}

void Logger::SerialLogger::received(char command[]) {
    if (Serial && on) {
        Serial.println("### SERIAL ###");
        Serial.print("RECEIVED: \"");
        Serial.print(command);
        Serial.println("\"");
    }
}
void Logger::SerialLogger::ping() {
    if (Serial && on) {
        Serial.println("PING");
        Serial.println("### END SERIAL ###\n");
    }
}
void Logger::SerialLogger::stop_motors() {
    if (Serial && on) {
        Serial.println("STOP MOTORS");
        Serial.println("### END SERIAL ###\n");
    }
}
void Logger::SerialLogger::set_powers() {
    if (Serial && on) {
        Serial.println("SET POWERS");
        Serial.print("L: ");
        Serial.print(target_powers[0]);
        Serial.print(", R: ");
        Serial.println(target_powers[1]);
        Serial.println("### END SERIAL ###\n");
    }
}
void Logger::SerialLogger::set_velocities() {
    if (Serial && on) {
        Serial.println("SET VELOCITIES");
        Serial.print("VX: ");
        Serial.print(target_robot_velocities_m[0]);
        Serial.print(", VT: ");
        Serial.println(target_robot_velocities_m[1]);
        Serial.println("### END SERIAL ###\n");
    }
}
void Logger::SerialLogger::set_pid() {
    if (Serial && on) {
        Serial.println("SET PID");
        Serial.print("Kp: ");
        Serial.print(Kp);
        Serial.print(", Ki: ");
        Serial.print(Ki);
        Serial.print(", Kd: ");
        Serial.println(Kd);
        Serial.println("### END SERIAL ###\n");
    }
}
void Logger::SerialLogger::request_odometry(int x_mm, int y_mm, int th_mm) {
    if (Serial && on) {
        Serial.println("REQUEST ODOMETRY");
        Serial.print("X: ");
        Serial.print(x_mm);
        Serial.print(", Y: ");
        Serial.print(y_mm);
        Serial.print(", TH: ");
        Serial.println(th_mm);
        Serial.println("### END SERIAL ###\n");
    }
}
void Logger::SerialLogger::reset_odometry() {
    if (Serial && on) {
        Serial.println("RESET ODOMETRY");
        Serial.println("### END SERIAL ###\n");
    }
}
void Logger::SerialLogger::set_odometry() {
    if (Serial && on) {
        Serial.println("SET ODOMETRY");
        Serial.print("X: ");
        Serial.print(actual_robot_position_u.x/1000);
        Serial.print(", Y: ");
        Serial.print(actual_robot_position_u.y/1000);
        Serial.print(", TH: ");
        Serial.println(actual_robot_position_u.th/1000);
        Serial.println("### END SERIAL ###\n");
    }
}
void Logger::SerialLogger::complete_odometry(int x_mm, int y_mm, int th_mm, int v_lin, int v_ang) {
    if (Serial && on) {
        Serial.println("ODOMETRY DEBUG REQUEST");
        Serial.print("X: ");
        Serial.print(x_mm);
        Serial.print(", Y: ");
        Serial.print(y_mm);
        Serial.print(", TH: ");
        Serial.print(th_mm);
        Serial.print(", VX: ");
        Serial.print(v_lin);
        Serial.print(", VT: ");
        Serial.println(v_ang);
        Serial.println("### END SERIAL ###\n");
    }
}
void Logger::SerialLogger::set_path() {
    if (Serial && on) {
        Serial.println("SET PATH");
        Serial.print("N: ");
        Serial.println(target_position_queue.count);
        for (int i = 0; i < target_position_queue.count; i++) {
            Serial.print(i);
            Serial.print(")  X: ");
            Serial.print(target_position_queue.queue[i].x);
            Serial.print(", Y: ");
            Serial.print(target_position_queue.queue[i].y);
            Serial.print(", TH: ");
            Serial.println(target_position_queue.queue[i].th);
        }
        Serial.println("### END SERIAL ###\n");
    }
}
void Logger::SerialLogger::append_path() {
    if (Serial && on) {
        Serial.println("APPEND PATH");
        Serial.print("N: ");
        Serial.println(target_position_queue.count);
        for (int i = 0; i < target_position_queue.count; i++) {
            Serial.print(i);
            Serial.print(")  X: ");
            Serial.print(target_position_queue.queue[i].x);
            Serial.print(", Y: ");
            Serial.print(target_position_queue.queue[i].y);
            Serial.print(", TH: ");
            Serial.println(target_position_queue.queue[i].th);
        }
        Serial.println("### END SERIAL ###\n");
    }
}
void Logger::SerialLogger::invalid_command() {
    if (Serial && on) {
        Serial.println("INVALID COMMAND");
        Serial.println("### END SERIAL ###\n");
    }
}


void Logger::OdometryLogger::log(
    long ticks_l, long ticks_r, long d_ticks_l, long d_ticks_r, long d_time_us, 
    long dL_um, long dR_um, long dS_um, long dT_urad) 
{
    if (Serial && is_time && odometry_on) {
        Serial.println("### ODOMETRY ###")

        Serial.println("TICKS TOT "); 
        Serial.print("L: ")
        Serial.print(ticks_l); 
        Serial.print(", R: ")
        Serial.println(ticks_r); 

        Serial.println("DELTA TICKS:"); 
        Serial.print("L: ")
        Serial.print(d_ticks_l); 
        Serial.print(", R: ")
        Serial.println(d_ticks_r);

        Serial.print("TIME: "); 
        Serial.print(d_time_us/1000); 
        Serial.println("ms");

        Serial.print("dL ");    Serial.print(dL_um); 
        Serial.print(", dR ");  Serial.println(dR_um);

        Serial.print("dS ");    Serial.print(dS_um);
        Serial.print(", dT ");  Serial.println(dT_urad);

        Serial.print("Wheels (mm/s): L ");
        Serial.print(actual_wheel_velocities_m[0]);
        Serial.print(", R");
        Serial.println(actual_wheel_velocities_m[1]);

        Serial.print("Velocity: VX ");
        Serial.print(actual_robot_velocities_m[0]);
        Serial.print(", VTHETA (mrad/s) ");
        Serial.println(actual_robot_velocities_m[1]);

        Serial.print("Position (mm): X "); 
        Serial.print(actual_robot_position_u.x/1000.0); 
        Serial.print(", Y "); 
        Serial.print(actual_robot_position_u.y/1000.0); 
        Serial.print(", TH "); 
        Serial.println(actual_robot_position_u.th/1000.0);

        Serial.println("### END ODOMETRY ###\n");
    }
}

void Logger::ControlLogger::log_change_target_position(int dist_mm) {
    if (Serial && is_time && on) {
        Serial.println("CHANGE TARGET POSITION");
        Serial.println("NEW TARGET:");
        Serial.print("X ");
        Serial.print(target_robot_position_m.x);
        Serial.print(", Y ");
        Serial.print(target_robot_position_m.y);
        Serial.print(", TH ");
        Serial.println(target_robot_position_m.th);
        Serial.print("DISTANCE: ");
        Serial.print(dist_mm);
        Serial.print(" / ");
        Serial.println(THRESH_DIST_MM);
        Serial.println();
    }
}
void Logger::ControlLogger::log_heading_adjustment(int dist_mm, int heading_error) {
    if (Serial && is_time && on) {
        Serial.println("HEADING ADJUSTMENT");
        Serial.print("DISTANCE: ");
        Serial.print(dist_mm);
        Serial.print(" mm, HEADING ERROR: ");
        Serial.print(heading_error);
        Serial.println(" mrad");
        Serial.println();
    }

}
void Logger::ControlLogger::log_alignment_adjustment(int dist_mm, int align_error) {
    if (Serial && is_time && control_on) {
        Serial.println("ALIGNMENT ADJUSTMENT");
        Serial.print("DISTANCE: ");
        Serial.print(dist_mm);
        Serial.print(" mm, ALIGNMENT ERROR: ");
        Serial.print(align_error);
        Serial.println(" mrad");
        Serial.println();
    }
}
void Logger::ControlLogger::log_velocity() {
    if (Serial && is_time && control_on) {
        Serial.println("TARGET VELOCITIES");
        Serial.print("VX:   ")
        Serial.print(target_robot_velocities_m[0]);
        Serial.print(" [mm/s],     VT:   ");
        Serial.print(target_robot_velocities_m[1]);
        Serial.println(" [mrad/s]");

        Serial.println("\nTARGET WHEEL VELOCITIES");
        Serial.print("L:    ")
        Serial.print(target_wheel_velocities_m[0]);
        Serial.print(" [mm/s],     R:    ");
        Serial.print(target_wheel_velocities_m[1]);
        Serial.println(" [mm/s]");

        Serial.println("\nPID VALUES");
        Serial.print("Kp:   ")
        Serial.print(Kp);
        Serial.print(",     Ki:   ");
        Serial.print(Ki);
        Serial.print(",     Kd:   ");
        Serial.println(Kd);
        Serial.println();
        
    }
}
void Logger::ControlLogger::log_power() {
    if (Serial && is_time && control_on) {
        Serial.println("TARGET POWERS");
        Serial.print("L:    ")
        Serial.print(target_powers[0]);
        Serial.print(",     R:    ");
        Serial.println(target_powers[1]);
        Serial.println();
    }
}