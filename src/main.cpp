#include "main.h"
#include "lemlib/api.hpp"
#include "globals.h"
#include "subsystems/drivetrain.h"
#include "subsystems/intake.h"
#include "routines/autons.h"
#include "routines/skills.h"
#include "subsystems/macros.h"


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
    pros::lcd::initialize();

    chassis.calibrate();


    left_motor_group.tare_position_all();
    right_motor_group.tare_position_all();
    intake_motor.tare_position();
    intake_motor2.tare_position();

    
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
    
}

// ===== DISABLED FUNCTION =====
void disabled() {

}


// ===== COMPETITION INITIALIZE =====
void competition_initialize() {

}

// ===== AUTONOMOUS FUNCTION =====
void autonomous() {

}



// ===== OPCONTROL FUNCTION =====
void opcontrol() {
    while (true) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            autonomous();
        }


        // L1: reverse main intake
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake1_reverse = !intake1_reverse;
            intake1_forward = false;

            if (intake1_reverse) {
                intake_reverse_full();
            } else {
                intake_stop();
            }
            pros::delay(100);
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
        

        // Pneumatics
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
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
