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
bool pne4 = false;
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
    cascade_motor.tare_position();

    cascade_init();

    left_motor_group.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    right_motor_group.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    small_motor.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    // Build the brain UI and start refreshing it. Runs in the background so
    // it stays live during disabled, autonomous and driver control.
    screen_init();
    screen_task();

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
    auton3();
    //auton2();
    //left4_1();
    //skills_auton();
}

// ===== OPCONTROL FUNCTION =====
void opcontrol() {
    // Rumble cues. Match length and the two warning points, in seconds
    // remaining; change a cue time here and nothing else needs touching.
    constexpr int DRIVER_DURATION_S = 105;
    constexpr int CUE_1_REMAINING_S = 35;
    constexpr int CUE_2_REMAINING_S = 15;

    // One flag per cue so each rumble fires on exactly one loop iteration.
    bool cue1_fired = false;
    bool cue2_fired = false;

    // Start the match clock shown on the STATUS tab. PROS has no match clock,
    // so elapsed driver time is measured from here.
    screen_match_start();

    left_motor_group.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    right_motor_group.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    // Match clock for the rumble cues below. Stamped once, here, so the cues
    // are measured from the moment driver control actually starts.
    uint32_t start = pros::millis();

    while (true) {
        // Driver-control rumble cues. Nothing blocks: this reads the clock and
        // only calls rumble() on the single iteration where a cue fires, so it
        // rides the loop's existing 25 ms delay. Signed math keeps the
        // remaining-time comparison sane after the match clock runs past zero.
        int32_t elapsed_s = static_cast<int32_t>((pros::millis() - start) / 1000);
        int32_t remaining_s = DRIVER_DURATION_S - elapsed_s;
        if (!cue1_fired && remaining_s <= CUE_1_REMAINING_S) {
            cue1_fired = true;
            controller.rumble("-");
        }
        if (!cue2_fired && remaining_s <= CUE_2_REMAINING_S) {
            cue2_fired = true;
            controller.rumble("- -");
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            //autonomous();
        }


        

        // L2: spin both 5.5W motors at full speed while held
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            small_motor.move_voltage(12000);
        } else {
            small_motor.move_voltage(0);
        }

        // Up: intake forward (was on L2)
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            intake1_forward = !intake1_forward;

            if (intake1_forward) {
                intake1_reverse = false;
                intake_forward_full();
            } else {
                intake_stop();
            }
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
        // B: pneumatics solenoid on port H
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            pne4 = !pne4;
            pneD.set_value(pne4);
        }

        // drive controls
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX);

        pros::delay(25);

        
    }
}
