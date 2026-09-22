#include "main.h"
#include "liblvgl/llemu.hpp"
#include "motion/chassis.h"
#include "globals.h"
#include "autonomous.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "selector.h"

void initialize() {
	selector::init();
	controller1.set_text(1, 0, "test1");
	chassis.Calibrate();
	controller1.set_text(1, 0, "test2");
	chassis.GetOdometry().StartUpdating();
	controller1.set_text(1, 0, "test3");
	lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	controller1.set_text(1, 0, "test4");
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
	while (true) {
		chassis.CentricArcade(controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), false);
		if (controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intake.move(127);
		}
		else if (controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intake.move(-127);
		}
		else {
			intake.move(0);
		}


		if (controller2.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			clamp.set_value(true);
		}
		else if (controller2.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			clamp.set_value(false);
		}
		
		if (controller2.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			lift.move(127);
		} 
		else if (controller2.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			lift.move(-80);
		}
		else {
			lift.move(0);
		}
		pros::delay(20);
	}
}