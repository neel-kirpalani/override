#include "main.h"
#include "lemlib/api.hpp"
#include "globals.h"
#include "subsystems/drivetrain.h"
#include "routines/autons.h"
#include "routines/skills.h"
#include "subsystems/cascade.h"
#include "subsystems/screen.h"
#include "mvlib/api.hpp"
#include "mvlib/Optional/lemlib.hpp"


bool pne1 = false;

void disabled() {

}

void competition_initialize(){

}

void initialize() {
    chassis.calibrate();
    
    left_motor_group.tare_position_all();
    right_motor_group.tare_position_all();
    cascade_motor.tare_position();

    cascade_init();

    left_motor_group.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    right_motor_group.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    // Build the brain UI and start refreshing it. Runs in the background so
    // it stays live during disabled, autonomous and driver control.
    screen_init();
    screen_task();


    auto& logger = mvlib::Logger::getInstance();
    mvlib::setOdom(&chassis);
    logger.setRobot({
        .leftDrivetrain = &left_motor_group,
        .rightDrivetrain = &right_motor_group
    });
    logger.start();

}

void autonomous() {
    
}

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
    
        

        

        
        

        

        // cascade controls
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


        // drive controls
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX);

        pros::delay(25);

        
    }
}
