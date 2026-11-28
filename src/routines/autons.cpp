#include "main.h"
#include "globals.h"
#include "subsystems/drivetrain.h"
#include "routines/autons.h"
#include "subsystems/cascade.h"

void angular_PID_tune() {
    chassis.setPose(0, 0, 0);
    pros::delay(200);
    chassis.turnToHeading(90, 2000);
    pros::delay(200);
    chassis.turnToHeading(180, 2000);
    pros::delay(200);
    chassis.turnToHeading(0, 2000);   
}


void lateral_PID_tune() {
    chassis.setPose(0, 0, 0);
    pros::delay(500);
    chassis.moveToPoint(0, 24, 10000);
}

void three_pin(){
    chassis.moveToPose(46.7, 12.5, 0, 4000, {.forwards = true, .lead = 0.403, .maxSpeed = 127, .minSpeed = 0}, false);
    pros::delay(100);
    chassis.moveToPoint(46.7, 16.8, 4000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0}, false);
    chassis.moveToPoint(46.7, 8.1, 4000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0}, false);
    chassis.turnToHeading(303, 2000, {.maxSpeed = 127, .minSpeed = 0}, false);
    chassis.moveToPoint(29.7, 19.2, 4000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0}, false);
    chassis.moveToPoint(39.6, 23.4, 4000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0}, false);
    chassis.turnToHeading(90, 2000, {.maxSpeed = 127, .minSpeed = 0}, false);
    chassis.moveToPoint(28.2, 28.3, 4000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0}, false);
    chassis.turnToHeading(45, 2000, {.maxSpeed = 127, .minSpeed = 0}, false);
    chassis.moveToPoint(41.2, 41.2, 4000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0}, false);
    chassis.turnToHeading(166.18, 2000, {.maxSpeed = 127, .minSpeed = 0}, false);
    chassis.moveToPoint(44.8, 30, 4000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0}, false);
}

void two_wall_pins(){ 

}

void secondary_one_pin(){
    
}

void primary_one_pin(){     
    
}
