#include "autonomous.h"
#include "globals.h"
#include "main.h"
#include "motion/chassis.h"
#include "motion/clawPivotController.h"
#include "pros/screen.hpp"
#include "selector.h"

//drivebase length = 15.75, drivebase width = 15, half length = 7.875, half width = 7.5
void Auto1() {
    lift.move_relative(270, 100);
    chassis.MoveToPosition(-3, 0, 45, 1);
    pros::delay(250);
    chassis.MoveToPosition(-3, -7, 45, 0.75);
    clawPivotController.getControllerData().setTargetHeading(clawPivotController.getMotor().get_position() - 110);
    chassis.MoveToPosition(6, -7, 45, 0.75);
    if (selector::auton.team == "red") {
        while (!isInColour(optical.get_hue(), 0, 35)) {
            toggleMech();
            pros::delay(25);
        }
    }
    else if (selector::auton.team == "blue") {
        while (!isInColour(optical.get_hue(), 160, 200)) {
            toggleMech();
            pros::delay(25);
        }
    }
    toggles.move(0);
    chassis.GetOdometry().SetPosition(0, 0, chassis.GetOdometry().GetHeading() - 45);
    chassis.MoveToPosition(-14, 0, 0, 1.5);
    chassis.MoveToPosition(-14, 15, 0, 0.75);
    lift.move_relative(-270, 50);
    clawPivotController.getControllerData().setTargetHeading(clawPivotController.getMotor().get_position() + 15);
    pros::delay(250);
    clamp.retract();
    pros::delay(250);
}

void Auto2() {
    lift.move_relative(270, 100);
    chassis.MoveToPosition(3, 0, -135, 1);
    pros::delay(250);
    chassis.MoveToPosition(3, 7, -135, 0.75);
    clawPivotController.getControllerData().setTargetHeading(clawPivotController.getMotor().get_position() - 110);
    chassis.MoveToPosition(6, 7,  -135, 0.75);
    if (selector::auton.team == "red") {
        while (!isInColour(optical.get_hue(), 0, 35)) {
            toggleMech();
            pros::delay(25);
        }
    }
    else if (selector::auton.team == "blue") {
        while (!isInColour(optical.get_hue(), 160, 200)) {
            toggleMech();
            pros::delay(25);
        }
    }
    toggles.move(0);
    chassis.GetOdometry().SetPosition(0, 0, chassis.GetOdometry().GetHeading());
    chassis.MoveToPosition(0, 0, 0, 1.25);
    chassis.MoveToPosition(14, 0, 0, 1.5);
    chassis.MoveToPosition(14, -15, 0, 0.75);
    lift.move_relative(-270, 50);
    clawPivotController.getControllerData().setTargetHeading(clawPivotController.getMotor().get_position() + 15);
    pros::delay(250);
    clamp.retract();
    pros::delay(250);
}

void Skills() {
    lift.move_relative(270, 100);
    chassis.MoveToPosition(-3, 0, 45, 1);
    pros::delay(250);
    chassis.MoveToPosition(-3, -7, 45, 0.75);
    clawPivotController.getControllerData().setTargetHeading(clawPivotController.getMotor().get_position() - 110);
    chassis.MoveToPosition(6, -7, 45, 0.75);
    while (!isInColour(optical.get_hue(), 0, 35)) {
        toggleMech();
        pros::delay(25);
    }
    toggles.move(0);
    chassis.GetOdometry().SetPosition(0, 0, chassis.GetOdometry().GetHeading() - 45);
    chassis.MoveToPosition(-14, 0, 0, 1.5);
    chassis.MoveToPosition(-14, 15, 0, 0.75);
    lift.move_relative(-270, 50);
    clawPivotController.getControllerData().setTargetHeading(clawPivotController.getMotor().get_position() + 15);
    pros::delay(250);
    clamp.retract();
    pros::delay(250);
    chassis.GetOdometry().SetPosition(-14, 8.8, chassis.GetOdometry().GetHeading());
    chassis.MoveToPosition(-50, 0, 0, 1.25, 0.8);
}