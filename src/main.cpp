#include "main.h"
#include "lemlib/api.hpp"
#include "globals.h"
#include "subsystems/drivetrain.h"
#include "subsystems/intake.h"
#include "routines/autons.h"
#include "routines/skills.h"
#include "subsystems/macros.h"
#include "subsystems/cascade.h"
#include "subsystems/small_motor.h"
#include "subsystems/screen.h"
#include "mvlib/api.hpp"
#include "mvlib/Optional/lemlib.hpp"


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

// ===== CLAW DISTANCE SENSOR (driver-control auto-close) =====
#define CLAW_DIST_PORT 5
static pros::Distance clawDist(CLAW_DIST_PORT);

// UNVERIFIED assumption: false CLOSES pneA.
// I must confirm this on the robot before using auto-close.
constexpr bool CLAW_CLOSE_VALUE = false;

// Speed compensation — default OFF until I verify signed motor velocities.
constexpr bool CLAW_SPEED_COMP = true;
// Sign of forward RPM for left+right average. May be -1.0 or 1.0.
constexpr double CLAW_RPM_SIGN = 1.0;
constexpr int32_t CLAW_BASE_TRIGGER_MM = 130;
// 100 ms is an untested tuning value, NOT measured pneumatic closure time.
constexpr double CLAW_LEAD_MS = 50;
constexpr int32_t CLAW_MAX_TRIGGER_MM = 200;

// Must be above CLAW_MAX_TRIGGER_MM so a valid clear reading always re-arms.
constexpr int CLAW_CLEAR_MM = 240;

constexpr int CLAW_CONFIRM_READS = 1;
static bool claw_wait_clear = false;
static int claw_hits = 0;




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
    small_motor_init();
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

// ===== DISABLED FUNCTION =====
void disabled() {
    small_motor_stop();
}


// ===== COMPETITION INITIALIZE =====
void competition_initialize() {

}

// ===== AUTONOMOUS FUNCTION =====
// Pick the routine by changing the call below. There is no auton selector any
// more: build a separate program per auton and switch between them on the
// controller.
void autonomous() {
    //small_motor_stop(); // do not remove
    //auton3();
    //auton2();
    //left4_1();
    //skills_auton();
    auton4();
}

// ===== OPCONTROL FUNCTION =====
void opcontrol() {
    // Claw auto-close state: no object seen yet, and armed to close.
    claw_hits = 0;
    claw_wait_clear = false;

    small_motor_stop();
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
        bool l1_pressed_this_loop = false;

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
            //auton3();
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

            l1_pressed_this_loop = true;
            claw_hits = 0;
            if (pne1 != CLAW_CLOSE_VALUE) {
                claw_wait_clear = true;
            }
        }

        // Claw auto-close: driver control only. Non-blocking; rides the loop's
        // existing 25 ms delay. Manual L1 wins for this tick, and after a manual
        // open the claw re-arms only once the sensor reads clear.
        //
        // Speed compensation (off by default): estimate forward wheel speed from
        // motor RPM and add reciprocal lead distance.  The 2.74 conversion is an
        // approximate mm/s per motor RPM based on the existing 2.75-inch wheels
        // and 450/600 drivetrain ratio.  get_actual_velocity() reads the FIRST
        // motor of each group, not a group average.  This is a wheel-speed
        // estimate, not measured ground or pin-relative speed; slip and curved
        // driving affect it.  Reverse motion gets no extra lead after the sign
        // is correctly set, but the base trigger still operates.
        double left_rpm  = left_motor_group.get_actual_velocity();
        double right_rpm = right_motor_group.get_actual_velocity();
        int32_t claw_trigger_mm = CLAW_BASE_TRIGGER_MM;
        if (CLAW_SPEED_COMP && std::isfinite(left_rpm) && std::isfinite(right_rpm)) {
            double fwd_mm_s = (left_rpm + right_rpm) / 2.0 * 2.74 * CLAW_RPM_SIGN;
            if (fwd_mm_s < 0) fwd_mm_s = 0;
            double t = CLAW_BASE_TRIGGER_MM + fwd_mm_s * CLAW_LEAD_MS / 1000.0;
            if (!std::isfinite(t) || t < CLAW_BASE_TRIGGER_MM) t = CLAW_BASE_TRIGGER_MM;
            if (t > CLAW_MAX_TRIGGER_MM) t = CLAW_MAX_TRIGGER_MM;
            claw_trigger_mm = (int32_t)t;
        }

        // Temporary sign-check logging, at most twice per second.
        static uint32_t claw_dbg_last = 0;
        uint32_t claw_dbg_now = pros::millis();
        if (claw_dbg_now - claw_dbg_last >= 500) {
            claw_dbg_last = claw_dbg_now;
            printf("[claw-dbg] L %.1f R %.1f rpm\n", left_rpm, right_rpm);
        }

        int32_t mm = clawDist.get_distance();
        if (mm == PROS_ERR || mm <= 0) {
            claw_hits = 0;
            // Error/nonpositive reading: never treat it as object or clear.
        } else {
            if (mm >= CLAW_CLEAR_MM) {
                claw_wait_clear = false;
            }
            bool claw_open = (pne1 != CLAW_CLOSE_VALUE);
            if (!l1_pressed_this_loop && claw_open &&
                !claw_wait_clear && mm < claw_trigger_mm) {
                claw_hits++;
            } else {
                claw_hits = 0;
            }
            if (claw_hits >= CLAW_CONFIRM_READS) {
                pne1 = CLAW_CLOSE_VALUE;
                pneA.set_value(pne1);
                claw_hits = 0;
                claw_wait_clear = true;
                controller.rumble(".");
                printf("[claw] auto close at %d mm (trigger %d)\n", (int)mm, (int)claw_trigger_mm);
            }
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
