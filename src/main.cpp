#include "main.h"
#include "motion/chassis.h"
#include "globals.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"

void initialize() {
	pros::lcd::initialize();
	chassis.Calibrate();
	chassis.getOdometry().startUpdating();
	lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void disabled() {}

void competition_initialize() {
	chassis.Calibrate();
}

void autonomous() {}

void opcontrol() {
	while (true) {
		pros::lcd::print(1, "Left Y: %d", controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
		pros::lcd::print(2, "Left X: %d", controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
		pros::lcd::print(3, "Right Y: %d", controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
		pros::lcd::print(4, "Right X: %d", controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		chassis.CentricArcade(controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), true);
		if (controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			lift.move(127);
		} 
		else if (controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			lift.move(-63);
		}
		else {
			lift.move(0);
		}
		pros::delay(20);
	}
}