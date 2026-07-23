#include "main.h"
#include "motion/chassis.h"
#include "globals.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"

void initialize() {
	pros::lcd::initialize();
	opcontrol();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	while (true) {
		pros::lcd::print(1, "Left Y: %d", controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
		pros::lcd::print(2, "Left X: %d", controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
		pros::lcd::print(3, "Right Y: %d", controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
		pros::lcd::print(4, "Right X: %d", controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		chassis.RobotCentricArcade(controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		pros::delay(20);

		
	}
}