#include "main.h"
#include "motion/chassis.h"
#include "globals.h"
#include "autonomous.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "selector.h"

bool runIntakeAdjustment = true;

void releasePin() {
	runIntakeAdjustment = false;
	intake.move(-63);
	lift.move(127);
	pros::delay(500);
	intake.move(0);
	lift.move(0);
	runIntakeAdjustment = true;
}

void clampPin() {
	runIntakeAdjustment = false;
	intake.move(127);
	scoringPiston.set_value(false);
	pros::delay(250);
	scoringPiston.set_value(true);
	pros::delay(100);
	intake.move(0);
	runIntakeAdjustment = true;
}

void thread() {
	while (true) {
		if (runIntakeAdjustment) {
			if (controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
				lift.move(127);
			} 
			else if (controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
				lift.move(-63);
			}
			else {
				lift.move(0);
			}
			if (controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
				intake.move(127);
			}
			else if (controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
				intake.move(-127);
			}
			else {
				intake.move(0);
			}
		}
		pros::delay(20);
	}
}

void initialize() {
	selector::init();
	chassis.Calibrate();
	chassis.GetOdometry().StartUpdating();
	lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void disabled() {}

void competition_initialize() {
	chassis.Calibrate();
}

void autonomous() {
	if (selector::auton.position == "1") {
		Auto1();
	}
	else if (selector::auton.team == "skills") {
		Skills();
	}
	else {
		Auto2();
	}
}

void opcontrol() {
	pros::Task threadTask(thread);
	while (true) {
		chassis.CentricArcade(controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), false);
		if (controller1.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			pros::Task releasePinTask(releasePin);
			while (controller1.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
				pros::delay(20);
			}
		}
		if (controller1.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
			pros::Task clampPinTask(clampPin);
			while (controller1.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
				pros::delay(20);
			}
		}
		if (controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) > 80) {
			scoringPiston.set_value(true);
		}
		else if (controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) < -80) {
			scoringPiston.set_value(false);
		}
		pros::delay(20);
	}
}