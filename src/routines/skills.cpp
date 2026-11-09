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
    cascade_go_to(700);
    pros::delay(50);
    chassis.turnToHeading(77, 900, {}, true);
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

    

    chassis.moveToPoint(40, 41.7, 2000, {.maxSpeed=53});
    //chassis.moveToPoint(41.379, 41.39, 2000, {.maxSpeed=47});
    pros::delay(830);
    pneA.set_value(false);
    pros::delay(200);
    cascade_go_to(800);
    pros::delay(50);
    chassis.turnToHeading(166, 800, {});
    chassis.waitUntilDone();
    pros::delay(10);
    chassis.moveToPoint(43.809, 31.577, 1500, {.maxSpeed=50});
    pros::delay(800);
    cascade_go_to(0);
    pros::delay(200);
    // end of auton3 code (finished scoring 3rd pin/cup on alliance goal)
    
    // --------------------- MATCHLOADING STARTS HERE -----------------------------

    pneA.set_value(true); 


    // go to matchloader and grab 1st pin + cup and score it on alliance goal:
    pros::delay(300);
    chassis.moveToPose(40, 59.293, 180, 1500, {.forwards=false, .minSpeed=80});
    pros::delay(200);
    chassis.turnToHeading(90, 900);
    pros::delay(100);
    chassis.moveToPoint(56, 57.8, 2000, {.maxSpeed=40});
    pros::delay(1100);
    pneA.set_value(false); // grab first pin + cup from matchloader
    pros::delay(450);
    cascade_go_to(200);
    pros::delay(1000);
    chassis.moveToPoint(48.5, 59.293, 1100, {.forwards=false, .maxSpeed=60}, true);
    pros::delay(300);
    cascade_go_to(1100);
    chassis.waitUntilDone();
    pros::delay(20);
    chassis.turnToHeading(180, 850, {}, false);
    pros::delay(20);

    left_motor_group.move_voltage(-5000);
    right_motor_group.move_voltage(-5000);
    pros::delay(700);
    left_motor_group.move_voltage(-0);
    right_motor_group.move_voltage(-0);
    
    chassis.setPose(46.998, 66.1, 180); // reset position against field wall
    
    chassis.moveToPoint(46.998, 34.432, 2000, {.maxSpeed=40}); // drive towards goal
    pros::delay(1700);
    cascade_go_to(0); // bring cascade down to score
    pros::delay(700);
    pneA.set_value(true);
    pros::delay(450);
    
    // second pin + cup (from matchloader)
    chassis.moveToPoint(49, 59.293, 2000, {.forwards=false, .maxSpeed=60}, true);
    pros::delay(500);
    chassis.turnToHeading(90, 900);
    pros::delay(300);
    chassis.moveToPoint(55.5, 55.293, 2000, {.maxSpeed=40});
    pros::delay(1100);
    pneA.set_value(false); // grab  pin + cup from matchloader
    pros::delay(800);
    cascade_go_to(100);
    pros::delay(200);
    chassis.moveToPoint(47, 59.293, 1200, {.forwards=false, .maxSpeed=30}, true);
    pros::delay(500);
    cascade_go_to(2000);
    pros::delay(1000);

    chassis.waitUntilDone();
    cascade_stop();
    pros::delay(500);
    chassis.turnToHeading(180, 900, {}, false);
    pros::delay(500);
    left_motor_group.move_voltage(-5000);
    right_motor_group.move_voltage(-5000);
    pros::delay(700);
    left_motor_group.move_voltage(-0);
    right_motor_group.move_voltage(-0);
    chassis.setPose(46.998, 66.1, 180);
    chassis.moveToPoint(46.998, 34.432, 2000, {.maxSpeed=40});
    pros::delay(2200);
    cascade_go_to(-10);
    pros::delay(500);
    pneA.set_value(true);
    pros::delay(300);


    chassis.moveToPoint(46.998, 47.521, 1500, {.forwards=false, .maxSpeed=100});
    pros::delay(500);
    chassis.turnToHeading(225, 1000);
    pros::delay(500);
    chassis.moveToPoint(9.374, 9.445, 2000, {.maxSpeed=100});



    
    
    /*
    // --------------------- SECOND ALLIANCE GOAL SCORING STARTS HERE ---------------------

    // go and matchload a single pin
    chassis.moveToPoint(49, 59.293, 2000, {.forwards=false, .maxSpeed=60}, true);
    pros::delay(500);
    chassis.turnToHeading(90, 900);
    pros::delay(300);
    chassis.moveToPoint(61.7, 59.5, 2000, {.maxSpeed=40});
    pros::delay(1100);
    pneA.set_value(false); // grab single pin from matchloader
    pros::delay(800);
    cascade_go_to(27);
    pros::delay(200);
    chassis.moveToPoint(46, 60.293, 1200, {.forwards=false, .maxSpeed=30}, true);
    pros::delay(100);
    cascade_go_to(600);
    chassis.waitUntilDone();
    chassis.turnToHeading(242, 2000, {.maxSpeed=60});
    pros::delay(100);
    chassis.moveToPoint(31.627, 51.285, 1500, {.maxSpeed=40});
    pros::delay(1300);
    cascade_go_to(0); // score the single pin in the alliance goal
    pros::delay(200);
    pneA.set_value(true); // open claw
    pros::delay(300);
    */


    /*
    // go to matchloader again and grab a pin + cup to score on that second alliance goal
    chassis.moveToPoint(46, 58.293, 1200, {.forwards=false, .minSpeed=60});
    pros::delay(50);
    chassis.turnToHeading(90,900);
    pros::delay(300);
    chassis.moveToPoint(64, 60, 2000, {.maxSpeed=40});
    pros::delay(1100);
    pneA.set_value(false); // grab  pin + cup from matchloader
    pros::delay(1000);
    cascade_go_to(100);
    pros::delay(200);
    chassis.moveToPoint(47, 58.293, 1200, {.forwards=false, .maxSpeed=30}, true);
    pros::delay(100);
    cascade_go_to(800);
    pros::delay(1000);
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.turnToHeading(242, 900);
    pros::delay(100);
    chassis.moveToPoint(31.627, 50.285, 1500, {.maxSpeed=50});
    pros::delay(1500);
    cascade_go_to(0); // score the single pin in the alliance goal
    pros::delay(200);
    pneA.set_value(true); // open claw
    pros::delay(300);

    // go to matchloader again and grab a pin + cup to score on that same second alliance goal
    chassis.moveToPoint(46, 58.293, 1200, {.forwards=false, .minSpeed=60});
    pros::delay(50);
    chassis.turnToHeading(90,900);
    pros::delay(300);
    chassis.moveToPoint(64, 63, 2000, {.maxSpeed=40});
    pros::delay(1100);
    pneA.set_value(false); // grab  pin + cup from matchloader
    pros::delay(1000);
    cascade_go_to(100);
    pros::delay(200);
    chassis.moveToPoint(47, 58.293, 1200, {.forwards=false, .maxSpeed=30}, true);
    pros::delay(100);
    cascade_go_to(800);
    pros::delay(1000);
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.turnToHeading(242, 900);
    pros::delay(100);
    chassis.moveToPoint(31.627, 50.285, 1500, {.maxSpeed=50});
    pros::delay(1500);
    cascade_go_to(0); // score the single pin in the alliance goal
    pros::delay(200);
    pneA.set_value(true); // open claw
    pros::delay(300);
    
  
    // TOGGLE CHANGE STARTS HERE:

    chassis.moveToPoint(46, 58.293, 1200, {.forwards=false, .minSpeed=60});
    pros::delay(50);
    chassis.turnToHeading(90,900);
    pros::delay(300);
    chassis.moveToPoint(68, 60, 2000, {.maxSpeed=40});
    pros::delay(1100);
    pneA.set_value(false); // grab  pin + cup from matchloader
    pros::delay(1000);
    cascade_go_to(100);
    pros::delay(200);
    chassis.moveToPoint(47, 58.293, 1200, {.forwards=false, .maxSpeed=30}, true);
    pros::delay(100);
    cascade_go_to(800);
    pros::delay(1000);
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.turnToHeading(270, 900);
    pros::delay(100);
    chassis.moveToPoint(0.354, 62.478, 1500, {.maxSpeed=90});

    pros::delay(500);
    small_motor.move_relative(2000, 200);

    */







}
