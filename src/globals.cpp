#include "main.h"
#include "pros/motors.h"
#include "pros/optical.h"
#include "pros/adi.h"
#include "pros/imu.h"
#include "globals.h"

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);


// Drivebase 
pros::MotorGroup left_motor_group({-12,-11,13}, pros::MotorGearset::blue);
pros::MotorGroup right_motor_group({1, 2, -3}, pros::MotorGearset::blue);

pros::MotorGroup left_tracking_motors({-12, -11}, pros::MotorGearset::blue);
pros::MotorGroup right_tracking_motors({1,2}, pros::MotorGearset::blue);

lemlib::TrackingWheel left_track(&left_tracking_motors, lemlib::Omniwheel::NEW_275, -5.6, 1.5);
lemlib::TrackingWheel right_track(&right_tracking_motors, lemlib::Omniwheel::NEW_275, 5.6, 1.5);

// Intakes 
pros::MotorGroup intake_motor({9, -8}, pros::MotorGearset::blue);
pros::MotorGroup intake_motor2({-7}, pros::MotorGearset::blue);

// Sensors 


// Pneumatics 
pros::adi::DigitalOut pneA('A');
pros::adi::DigitalOut pneB('B');
pros::adi::DigitalOut pneC('C');

// IMU 
pros::Imu imu(10);

// Distance Sensors



