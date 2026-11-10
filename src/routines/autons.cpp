#include "main.h"
#include "globals.h"
#include "subsystems/drivetrain.h"
#include "subsystems/intake.h"
#include "routines/autons.h"
#include "subsystems/cascade.h"
#include "subsystems/small_motor.h"

// ===== ANGULAR PID TUNE FUNCTION =====
void angular_PID_tune() {
    chassis.setPose(0, 0, 0);
    pros::delay(500);
    chassis.turnToHeading(90, 4000);
    
    pros::delay(500);
    chassis.turnToHeading(180, 4000);
    pros::delay(500);
    chassis.turnToHeading(0, 4000);
    
}

// ===== LATERAL PID TUNE FUNCTION =====
void lateral_PID_tune() {
    
    chassis.setPose(0, 0, 0);
    pros::delay(500);
    chassis.moveToPoint(0, 24, 10000);
   
    /*
    pros::delay(500);
    chassis.turnToHeading(180, 1000);
    pros::delay(500);
    chassis.moveToPoint(0,0,3000);
    */

    

    /*
    pros::delay(200);
    chassis.turnToHeading(270, 4000, {.maxSpeed=60});
    pros::delay(500);
    //chassis.setPose(0, 24, 90);
    pros::delay(200);
    chassis.moveToPoint(-24, 24, 4000, {.maxSpeed=60});
    */
}

void left4_1(){ // not tested
    small_motor.move_voltage(12000);
    pros::delay(500);
    small_motor.move_voltage(0);
    chassis.setPose(63.65, 0, 270);
    cascade_go_to(250);
    pros::delay(10);
    
    chassis.moveToPoint(60, 0, 500, {.maxSpeed=80});
    pros::delay(2);
    chassis.moveToPoint(65, 0, 500, {.forwards=false, .minSpeed=120});
    pros::delay(2);
    
    chassis.moveToPoint(58, 0, 700, {.minSpeed=80}, true);
    pros::delay(10);
    cascade_go_to(500);
    chassis.waitUntilDone();
    chassis.turnToHeading(201, 550);
    pros::delay(5);
    chassis.moveToPoint(50.734, -16.363, 1200, {.maxSpeed=60});
    pros::delay(1000);
    cascade_go_to(0);
    pros::delay(375);
    pneA.set_value(true);
    pros::delay(300);
    //chassis.moveToPoint(54.256, -6.556, 775, {.forwards=false, .maxSpeed=110});
    //pros::delay(5);
    //chassis.turnToHeading(175, 700);
    //pros::delay(500);
    //chassis.setPose(54.256, -6.556, 175);

    chassis.moveToPose(58, 0, 50.6, 1500,{.forwards=false,.minSpeed=80});
    pros::delay(100);

    chassis.arcade(50, 0);
    pros::delay(500);
    chassis.arcade(0,0);




}

void auton2(){ // awp auton for stamford
    small_motor.move_voltage(12000);
    pros::delay(500);
    small_motor.move_voltage(0);
    chassis.setPose(63.65, 0, 270);
    cascade_go_to(250);
    pros::delay(10);
    
    chassis.moveToPoint(60, 0, 500, {.maxSpeed=80});
    pros::delay(2);
    chassis.moveToPoint(65, 0, 500, {.forwards=false, .minSpeed=120});
    pros::delay(2);
    chassis.moveToPoint(60, 0, 500, {.maxSpeed=80});
    pros::delay(2);
    chassis.moveToPoint(65, 0, 500, {.forwards=false,.minSpeed=120});
    pros::delay(2);
    chassis.moveToPoint(58, 0, 700, {.minSpeed=80}, true);
    pros::delay(10);
    cascade_go_to(400);
    chassis.waitUntilDone();
    chassis.turnToHeading(339, 550);
    pros::delay(5);
    chassis.moveToPoint(50.734, 16.363, 1200, {.maxSpeed=60});
    pros::delay(1000);
    cascade_go_to(0);
    pros::delay(375);
    pneA.set_value(true);
    pros::delay(300);
    chassis.moveToPoint(54.256, 6.556, 775, {.forwards=false, .maxSpeed=110});
    pros::delay(5);
    chassis.turnToHeading(300, 700);
    pros::delay(5);
    //chassis.moveToPoint(41.374, 13.566, 900, {.maxSpeed=100});
    //pros::delay(5);
    chassis.moveToPoint(32.7, 17.8, 1500, {.maxSpeed=65});  
    pros::delay(700);
    pneA.set_value(false);
    pros::delay(350);
    chassis.turnToHeading(77, 900, {}, true);
    cascade_go_to(700);
    chassis.waitUntilDone();
    pros::delay(20);
    chassis.moveToPoint(39.124, 21.265, 1000, {.maxSpeed=30});
    pros::delay(400);
    cascade_go_to(0);
    pros::delay(500);
    pneA.set_value(true);
    pros::delay(200);
    chassis.moveToPoint(23.38, 23.391, 700, {.forwards=false,.maxSpeed=120});
    pros::delay(5);
    chassis.turnToHeading(49, 800);
    pros::delay(10);
    //chassis.moveToPoint(35.586, 35.597, 900, {.maxSpeed=100});
    //pros::delay(5);

    

    chassis.moveToPoint(40, 41.5, 2000, {.maxSpeed=53});
    //chassis.moveToPoint(41.379, 41.39, 2000, {.maxSpeed=47});
    pros::delay(800);
    pneA.set_value(false);
    pros::delay(200);
    chassis.turnToHeading(300, 700, {}, true);
    cascade_go_to(450);
    chassis.waitUntilDone();
    chassis.moveToPoint(26, 43.5, 1500, {.maxSpeed=70});
    pros::delay(700);
    cascade_go_to(0);
    pros::delay(100);
    
    

    
    
}

void auton3(){ // main auton
    small_motor.move_voltage(12000);
    pros::delay(500);
    small_motor.move_voltage(0);
    chassis.setPose(63.65, 0, 270);
    cascade_go_to(250);
    pros::delay(10);
    
    chassis.moveToPoint(60, 0, 500, {.maxSpeed=80});
    pros::delay(2);
    chassis.moveToPoint(65, 0, 500, {.forwards=false, .minSpeed=120});
    pros::delay(2);
    chassis.moveToPoint(60, 0, 500, {.maxSpeed=80});
    pros::delay(2);
    chassis.moveToPoint(65, 0, 500, {.forwards=false,.minSpeed=120});
    pros::delay(2);
    chassis.moveToPoint(58, 0, 700, {.minSpeed=80}, true);
    pros::delay(10);
    cascade_go_to(400);
    chassis.waitUntilDone();
    chassis.turnToHeading(339, 550);
    pros::delay(5);
    chassis.moveToPoint(50.734, 16.363, 1200, {.maxSpeed=60});
    pros::delay(1000);
    cascade_go_to(0);
    pros::delay(375);
    pneA.set_value(true);
    pros::delay(300);
    chassis.moveToPoint(54.256, 6.556, 775, {.forwards=false, .maxSpeed=110});
    pros::delay(5);
    chassis.turnToHeading(300, 700);
    pros::delay(5);
    //chassis.moveToPoint(41.374, 13.566, 900, {.maxSpeed=100});
    //pros::delay(5);
    chassis.moveToPoint(32.7, 17.8, 1500, {.maxSpeed=65});  
    pros::delay(700);
    pneA.set_value(false);
    pros::delay(350);
    chassis.turnToHeading(77, 900, {}, true);
    cascade_go_to(700);
    chassis.waitUntilDone();
    pros::delay(20);
    chassis.moveToPoint(39.124, 21.265, 1000, {.maxSpeed=30});
    pros::delay(400);
    cascade_go_to(0);
    pros::delay(500);
    pneA.set_value(true);
    pros::delay(200);
    chassis.moveToPoint(23.38, 23.391, 700, {.forwards=false,.maxSpeed=120});
    pros::delay(5);
    chassis.turnToHeading(49, 800);
    pros::delay(10);
    //chassis.moveToPoint(35.586, 35.597, 900, {.maxSpeed=100});
    //pros::delay(5);

    

    
    chassis.moveToPoint(40, 41.5, 2000, {.maxSpeed=53});
    //chassis.moveToPoint(41.379, 41.39, 2000, {.maxSpeed=47});
    pros::delay(850);
    pneA.set_value(false);
    pros::delay(200);
    chassis.turnToHeading(166, 700, {}, true);
    cascade_go_to(800);
    chassis.waitUntilDone();
    pros::delay(10);
    chassis.moveToPoint(43.809, 31.577, 1500, {.maxSpeed=50});
    pros::delay(800);
    cascade_go_to(0);
    pros::delay(100);
}

void auton4(){ // 4 pin auto (uses wall pin + cup)

    // preload into alliance goal
    chassis.setPose(62.5, 0, 270);
    cascade_go_to(350);
    chassis.moveToPose(47, 19.5, 0, 1200, {.minSpeed=90},true);
    pros::delay(950);
    cascade_go_to(0);
    pros::delay(500);
    pneA.set_value(true);
    pros::delay(100);

    // wall yellow
    chassis.moveToPoint(47, 7.5, 1500, {.forwards=false, .minSpeed=80});
    pros::delay(10);
    chassis.turnToHeading(50.6, 900);
    pros::delay(10);
    chassis.moveToPoint(57, 16, 1500, {.maxSpeed=50});
    pros::delay(10);
    chassis.swingToHeading(90, lemlib::DriveSide::RIGHT, 900, {},true);
    pros::delay(100);
    pneA.set_value(false);
    chassis.waitUntilDone();
    pros::delay(10);

    left_motor_group.move_voltage(8000);
    right_motor_group.move_voltage(8000);
    pros::delay(700);
    left_motor_group.move_voltage(0);
    right_motor_group.move_voltage(0);

    cascade_go_to(500);
    pros::delay(150);
    chassis.turnToHeading(30, 900);
    pros::delay(20);
    chassis.moveToPoint(50, 0, 1500, {.forwards=false});
    pros::delay(20);
    chassis.turnToHeading(0, 900);
    pros::delay(20);
    chassis.moveToPoint(50, 12, 1500, {.maxSpeed=60});
    pros::delay(650);
    cascade_go_to(0);
    pros::delay(200);
    pneA.set_value(true);

    // stray yellow 1


    // stray yellow 2

    // stray yellow 3


    

    
    
    
}
