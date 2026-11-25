#include "main.h"
#include "globals.h"
#include "subsystems/drivetrain.h"
#include "routines/autons.h"
#include "subsystems/cascade.h"
#include "subsystems/small_motor.h"

void angular_PID_tune() {
    chassis.setPose(0, 0, 0);
    pros::delay(500);
    chassis.turnToHeading(90, 4000);
    pros::delay(500);
    chassis.turnToHeading(180, 4000);
    pros::delay(500);
    chassis.turnToHeading(0, 4000);   
}


void lateral_PID_tune() {
    chassis.setPose(0, 0, 0);
    pros::delay(500);
    chassis.moveToPoint(0, 24, 10000);
}

void three_pin(){
    

}

void two_wall_pins(){ 

}

void secondary_one_pin(){
    
}

void primary_one_pin(){     
    
}
