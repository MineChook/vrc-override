#include "autonomous.h"
#include "globals.h"
#include "main.h"
#include "motion/chassis.h"
#include "motion/clawPivotController.h"
#include "pros/screen.hpp"
#include "selector.h"

//drivebase length = 15.75, drivebase width = 15, half length = 7.875, half width = 7.5
void Auto1() {
    chassis.MoveToPosition(-2, 2, 0, 1);
    lift.move_relative(360, 100);
    chassis.MoveToPosition(-2, 2, 45, 1);
    chassis.MoveToPosition(2, -1, 45, 2.5);
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
    chassis.GetOdometry().SetPosition(0, 0, chassis.GetOdometry().GetHeading() - 45);
    chassis.MoveToPosition(-13.25,0, 0, 4);
    pros::delay(200);
    chassis.MoveToPosition(-13.25, 9, 0, 4);
    lift.move_relative(-360, 50);
    pros::delay(500);
    clamp.retract();
    pros::delay(250);
    chassis.MoveToPosition(-36, 0, 0, 4, 0.7);
    chassis.MoveToPosition(-36, 12, 0, 4, 0, 64);
    clamp.extend(); 
    lift.move_relative(450, 100);
    chassis.MoveToPosition(-20, 18.5, 90, 4, 0, 90);
    lift.move_relative(-450, 50);
    pros::delay(500);
    clamp.retract();
    pros::delay(250);
    chassis.MoveToPosition(-32, 18.5, 45, 4);
    pros::Task task {[=] {
        while (true) {
            if (clampDistance.get() <= distanceToClampCup) {
                clamp.set_value(true);
                task.remove();
            }
            pros::delay(20);
        }
    }};
    chassis.MoveToPosition(-19.7, 36.7, 45, 4);
    clamp.extend();
    lift.move_relative(720, 100);
    pros::delay(250);
    chassis.MoveToPosition(-13, 26.9, 180, 4);
    lift.move_relative(-720, 50);
    pros::delay(500);
    clamp.retract();
}

void Auto2() {
    chassis.MoveToPosition(0, 0, 90, 10);
}

void Skills() {
    lift.move_relative(270, 100);
    chassis.MoveToPosition(-3, 0, 45, 1);
    pros::delay(250);
    chassis.MoveToPosition(-3, -7, 45, 0.75);
    //clawPivotController.getControllerData().setTargetHeading(clawPivotController.getMotor().get_position() - 110);
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
    //clawPivotController.getControllerData().setTargetHeading(clawPivotController.getMotor().get_position() + 15);
    pros::delay(250);
    clamp.retract();
    pros::delay(250);
    chassis.GetOdometry().SetPosition(-14, 8.8, chassis.GetOdometry().GetHeading());
    chassis.MoveToPosition(-50, 0, 0, 1.25, 0.8);
}