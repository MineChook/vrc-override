#include "main.h"
#include "liblvgl/llemu.hpp"
#include "motion/chassis.h"
#include "globals.h"
#include "autonomous.h"
#include "motion/clawPivotController.h"
#include "pros/ai_vision.h"
#include "pros/device.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "selector.h"
#include <atomic>
#include <string>
#include <sys/types.h>
#include <thread>

bool isInColour(int current, int min, int max) {
	return current >= min && current <= max;
}

void controllerToggle() {
	if (controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
		toggles.move(25);
	}
	else if (controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
		toggles.move(-25);
	}
	else {
		toggles.move(5);
	}
}

void toggleMech() {
	if (toggleDistance.get_distance() < 50) {
		double hue = optical.get_hue();
		if (isInColour(hue, 40, 80)) {
			toggles.move(25);
		}
		else if (isInColour(hue, 0, 35)) {
			if (!(selector::auton.team == "red" || selector::auton.team == "skills")) {
				toggles.move(25);
			}
			else {
				controllerToggle();
			}
		}
		else if (isInColour(hue, 160, 200)) {
			if (!(selector::auton.team == "blue")) {
				toggles.move(25);
			}
			else {
				controllerToggle();
			}
		}
		else {
			controllerToggle();
		}
	}
	else {
		toggles.move(0);
	}
}

void holdMech() {
	while (true) {
		if (controller1.get_digital(pros::E_CONTROLLER_DIGITAL_B) || controller2.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
			toggleHold.toggle();
			pros::delay(500);
		}
		pros::delay(20);
	}
}

void initialize() {
	selector::init();
	chassis.Calibrate();
	chassis.GetOdometry().StartUpdating();
	lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	clawPivot.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	pros::delay(1000);
	clamp.set_value(true);
	//clawPivotController.StartLoop();
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
	pros::Task task(holdMech);
	bool firstTime = true;
	int16_t ticks = 0;
	while (true) {
		/*
		if (controller2.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			clawPivotController.getControllerData().setTargetHeading(clawPivotController.getControllerData().getTargetHeading() + 90);
			pros::delay(500);
			clamp.retract();
		}
		else if (controller2.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_A)) {
			clawPivotController.getControllerData().setTargetHeading(clawPivotController.getControllerData().getTargetHeading() - 90);
		}
		*/
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
			if (firstTime) {
				ticks = 5;
				firstTime = false;
			}
			else {
				ticks--;
			}
			if (ticks <= 0) {
				if (clampDistance.get() <= distanceToClampCup) {
					clamp.set_value(true);
				}
				ticks = 0;
			}
		}
		else {
			if (ticks > 0) {
				clamp.set_value(true);
			}
			firstTime = true;
		}
		if (controller2.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			clamp.set_value(false);
			firstTime = true;
		}
		else if (controller2.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
			if (clampDistance.get() <= distanceToClampPin) {
					clamp.set_value(true);
			}
			firstTime = true;
		}
		
		if (controller2.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			lift.move(127);
		} 
		else if (controller2.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			lift.move(-70);
		}
		else {
			lift.move(0);
		}
		toggleMech();
		pros::delay(20);
	}
}