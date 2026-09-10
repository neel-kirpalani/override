#include "main.h"
#include "lemlib/api.hpp"
#include "globals.h"
#include "drivetrain.h"

// Drivetrain config
lemlib::Drivetrain drivetrain(
    &left_motor_group,               // left motor group
    &right_motor_group,              // right motor group
    11.2,                          // track width
    lemlib::Omniwheel::NEW_275,      // wheel type
    400,                             // drivetrain rpm
    2                                // horizontal drift
);



// Odometry sensors
lemlib::OdomSensors odomSensors(
    &left_track,  // vertical tracking wheel 1
    &right_track,  // vertical tracking wheel 2
    nullptr,  // horizontal tracking wheel 1
    nullptr,  // horizontal tracking wheel 2
    &imu      // inertial sensor
);

// Lateral PID controller
lemlib::ControllerSettings lateral_controller(
    6,   // kP
    0,   // kI
    9,   // kD
    0,   // anti windup
    0,   // small error range (in)
    0,   // small error timeout (ms)
    0,   // large error range (in)
    0,   // large error timeout (ms)
    0    // max acceleration
);

// Angular PID controller
lemlib::ControllerSettings angular_controller(
    1.6,   // kP
    0,   // kI
    8,  // kD
    0,   // anti windup
    0,   // small error range (deg)
    0,   // small error timeout (ms)
    0,   // large error range (deg)
    0,   // large error timeout (ms)
    0    // max acceleration
);

// Global chassis object
lemlib::Chassis chassis(
    drivetrain,
    lateral_controller,
    angular_controller,
    odomSensors
);
