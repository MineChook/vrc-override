#include "main.h"
#include "liblvgl/llemu.hpp"
#include "motion/chassis.h"
#include "globals.h"
#include "autonomous.h"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "selector.h"
#include <string>

bool isInColour(int current, int min, int max) {
	return current >= min && current <= max;
}

void toggleMech() {
	double hue = optical.get_hue();
	if (optical.get_proximity() > 5077) {
		return;
	}
	if (isInColour(hue, 40, 80)) {
		toggles.move(75);
	}
	else if (isInColour(hue, 0, 35)) {
		if (!(selector::auton.team == "red")) {
			toggles.move(75);
		}
		else {
			toggles.move(0);
		}
	}
	else if (isInColour(hue, 160, 200)) {
		if (!(selector::auton.team == "blue")) {
			toggles.move(75);
		}
		else {
			toggles.move(0);
		}
	}
	else {
		toggles.move(0);
	}

	pros::delay(20);
}

void initialize() {
	selector::init();
	chassis.Calibrate();
	chassis.GetOdometry().StartUpdating();
	lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	pros::delay(1000);
	clamp.set_value(true);
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
		chassis.CentricArcade(-controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), -controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), false);
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
		toggleMech();
		pros::delay(20);
	}
}