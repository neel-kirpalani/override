#include "main.h"
#include "pros/motors.h"
#include "pros/optical.h"
#include "pros/adi.h"
#include "pros/imu.h"
#include "globals.h"

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);


// Drivebase 
pros::MotorGroup left_motor_group({9,-10,-8}, pros::MotorGearset::blue);
pros::MotorGroup right_motor_group({1, 2, -3}, pros::MotorGearset::blue);

pros::MotorGroup left_tracking_motors({-10, -8}, pros::MotorGearset::blue);
pros::MotorGroup right_tracking_motors({1,2}, pros::MotorGearset::blue);

lemlib::TrackingWheel left_track(&left_tracking_motors, lemlib::Omniwheel::NEW_275, -5.6, 400);
lemlib::TrackingWheel right_track(&right_tracking_motors, lemlib::Omniwheel::NEW_275, 5.6, 400);

// Intakes 
pros::MotorGroup intake_motor({9, -8}, pros::MotorGearset::blue);
pros::MotorGroup intake_motor2({-7}, pros::MotorGearset::blue);

// Cascade
pros::MotorGroup cascade_motor({14, -15}, pros::MotorGearset::green, pros::MotorUnits::degrees);

// 5.5W motors - CHANGE THESE PORTS to wherever they are plugged in.
// 5.5W motors are green cartridges (200 rpm).
pros::MotorGroup small_motor({-4, 7}, pros::MotorGearset::green);

// Sensors 


// Pneumatics 
pros::adi::DigitalOut pneA('A');
pros::adi::DigitalOut pneB('B');
pros::adi::DigitalOut pneD('H');

// IMU 
pros::Imu imu(21);

// Distance Sensors



