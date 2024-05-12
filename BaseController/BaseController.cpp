#include "BaseController.h"

BaseController::BaseController(
    int mr_pwm, int mr_in1, int mr_in2, int mr_enc_a, int mr_enc_b,
    int ml_pwm, int ml_in1, int ml_in2, int ml_enc_a, int ml_enc_b,
    int s1_trig, int s1_echo,
    float wheel_radius, float wheel_distance, float counts_per_rev
) {
    _motor_right = MotorController(mr_pwm, mr_in1, mr_in2, mr_enc_a, mr_enc_b, counts_per_rev);
    _motor_left = MotorController(ml_pwm, ml_in1, ml_in2, ml_enc_a, ml_enc_b, counts_per_rev);
    _sensor1 = DistanceSensor(s1_trig, s1_echo);
    _wheel_radius = wheel_radius;
    _wheel_distance = wheel_distance;
}

state BaseController::updateState(unsigned float linear_velocity, float angular_velocity) {
    // linear_velocity in m/s
    // angular_velocity in round/s (0 front positive right)

    // calculate motors ancgular velocities
    float dist;
    float goal_rps_rigth, goal_rps_left; 
    float current_rps_right, current_rps_left;

    // update sensor data
    dist = _sensor1.updateDistance();

    if (dist < DIST_TRESHOLD) {
        goal_rps_rigth = 0;
        goal_rps_left = 0;
    } else {
        // calculate wheels angular velocities
        goal_rps_left = (linear_velocity - angular_velocity * _wheel_distance / 2) / _wheel_radius;
        goal_rps_rigth = (linear_velocity + angular_velocity * _wheel_distance / 2) / _wheel_radius;
    }
    
    // update motor velocity
    current_rps_right = _motor_right.updateVelocity(goal_rps_rigth);
    current_rps_left = _motor_left.updateVelocity(goal_rps_left);

    // update odometry
    _state = _updateOdometry(current_rps_right, current_rps_left);

    return _state;
}

state BaseController::_updateOdometry(float rps_right, float rps_left) {
    // rps in round/s
    // return state
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
