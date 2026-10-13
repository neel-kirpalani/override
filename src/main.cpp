#include "main.h"
#include "lemlib/api.hpp"
#include "globals.h"
#include "subsystems/drivetrain.h"
#include "subsystems/intake.h"
#include "routines/autons.h"
#include "routines/skills.h"
#include "subsystems/macros.h"
#include "subsystems/cascade.h"
#include "subsystems/screen.h"


// ===== GLOBAL VARIABLES =====
bool intake1 = false;
bool intake2 = false;
bool pne1 = false;
bool pne2 = false;
bool pne3 = false;
bool intake1_forward = false;
bool intake1_reverse = false;
bool intake2_forward = false;
bool intake2_reverse = false;
bool auto_intake_active = false;




void initialize() {
    chassis.calibrate();


    left_motor_group.tare_position_all();
    right_motor_group.tare_position_all();
    intake_motor.tare_position();
    intake_motor2.tare_position();

    cascade_init();

    left_motor_group.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    right_motor_group.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

    // Build the brain UI and start refreshing it. Runs in the background so
    // it stays live during disabled, autonomous and driver control.
    screen_init();
    pros::Task screenTask([]() {
        screen_task();
    }, TASK_PRIORITY_DEFAULT, 8192, "screen");

}

// ===== DISABLED FUNCTION =====
void disabled() {

}


// ===== COMPETITION INITIALIZE =====
void competition_initialize() {

}

// ===== AUTONOMOUS FUNCTION =====
// Pick the routine by changing the call below. There is no auton selector any
// more: build a separate program per auton and switch between them on the
// controller.
void autonomous() {
    auton3();          // <-- change this to the routine you want
}

// ===== OPCONTROL FUNCTION =====
void opcontrol() {
    // Start the match clock shown on the STATUS tab. PROS has no match clock,
    // so elapsed driver time is measured from here.
    screen_match_start();

    left_motor_group.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    right_motor_group.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    while (true) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            autonomous();
        }


        

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake1_forward = !intake1_forward;
            intake1_forward = false;

            if (intake1_forward) {
                intake_forward_full();
            } else {
                intake_stop();
            }
            pros::delay(100);
        }
        

        // Cascade controls. The lift control loop runs on its own task, so
        // there is nothing to tick here - only the driver's inputs.
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            cascade_home();
        }

        if (cascade_homing()) {
            // Return-to-bottom is in charge until it finishes
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            cascade_up();
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            cascade_down();
        } else {
            cascade_stop();
        }

        // Pneumatics
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            pne1 = !pne1;
            pneA.set_value(pne1);
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            pne3 = !pne3;
            pneC.set_value(pne3);
        }

        // drive controls
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX);

        pros::delay(25);

        
    }
}
