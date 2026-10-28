#include "main.h"
#include "globals.h"
#include "subsystems/drivetrain.h"
#include "subsystems/intake.h"
#include "routines/skills.h"
#include "subsystems/cascade.h"
#include "subsystems/small_motor.h"


void skills_auton() {
    // auton3 code first
    small_motor.move_voltage(12000);
    //pros::delay(500);
    //small_motor.move_voltage(0);
    chassis.setPose(63.65, 0, 270);
    cascade_go_to(250);
    pros::delay(10);
    small_motor.move_voltage(0);
    
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
    cascade_go_to(500);
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
    chassis.moveToPoint(35.273, 16.696, 1200, {.maxSpeed=65});  
    pros::delay(740);
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

    

    chassis.moveToPoint(40, 40, 2000, {.maxSpeed=50});
    //chassis.moveToPoint(41.379, 41.39, 2000, {.maxSpeed=47});
    pros::delay(920);
    pneA.set_value(false);
    pros::delay(150);
    chassis.turnToHeading(166, 700, {}, true);
    cascade_go_to(800);
    chassis.waitUntilDone();
    pros::delay(10);
    chassis.moveToPoint(43.809, 31.577, 1500, {.maxSpeed=50});
    pros::delay(800);
    cascade_go_to(0);
    pros::delay(300);
    // end of auton3 code (finished scoring 3rd pin/cup on alliance goal)
    pneA.set_value(true);

    // go to matchloader and grab 1st pin + cup
    pros::delay(500);
    chassis.moveToPose(40, 58.992, 0, 1500, {.forwards=false, .minSpeed=60});
    pros::delay(500);
    chassis.turnToHeading(90, 900);
    pros::delay(500);
    chassis.moveToPoint(61, 58.992, 1200, {.maxSpeed=30});
    pros::delay(400);
    pneA.set_value(false); // grab first pin + cup from matchloader
    pros::delay(500);
    cascade_go_to(75);
    pros::delay(200);
    chassis.moveToPoint(46.938, 58.992, 1200, {.forwards=false, .maxSpeed=60}, true);
    pros::delay(100);
    cascade_go_to(1075);
    chassis.waitUntilDone();
    pros::delay(200);
    chassis.turnToHeading(180, 900);
    pros::delay(500);
    chassis.moveToPoint(46.938, 31.577, 1500, {.maxSpeed=40}); // go to goal
    pros::delay(500);
    cascade_go_to(0); // bring cascade down to score
    pros::delay(500);
    
    pneA.set_value(true); // release pin + cup on top of existing stack

    
  






}
