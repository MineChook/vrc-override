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
    chassis.MoveToPosition(-13.25, 0, 0, 4);
    pros::delay(200);
    chassis.MoveToPosition(-13.25, 9, 0, 1);
    lift.move_relative(-270, 50);
    pros::delay(500);
    clamp.retract();
    pros::delay(500);
    lift.move_relative(270, 127);
    chassis.MoveToPosition(-13.25, 5, 0, 4);
    lift.move_relative(-270, 50);
    chassis.MoveToPosition(-38, 0, 0, 4.5, 0.7);
    pros::Task taskCup {[&taskCup] {
        while (true) {
            if (clampDistance.get() <= distanceToClampCup) {
                clamp.set_value(true);
                taskCup.suspend();
            }
            pros::delay(20);
        }
    }};
    chassis.MoveToPosition(-38, 12, 0, 2, 0, 64);
    lift.move_relative(450, 100);
    chassis.MoveToPosition(-20, 18.5, 90, 2.75, 0, 90);
    lift.move_relative(-360, 50);
    pros::delay(500);
    clamp.retract();
    pros::delay(250);
    chassis.MoveToPosition(-32, 18.5, 45, 4);
    lift.move_relative(-90, 100);
    taskCup.resume();
    chassis.MoveToPosition(-18.5, 38, 45, 4, 0, 64);
    lift.move_relative(720, 100);
    pros::delay(250);
    chassis.MoveToPosition(-12.25, 27.25, 180, 4);
    lift.move_relative(-360, 50);
    pros::delay(500);
    clamp.retract();
    pros::delay(250);
    chassis.MoveToPosition(-12.25, 53.25, 180, 4);
    chassis.MoveToPosition(-12.25, 53.25, 90, 4);
    lift.move_relative(-360, 50);
}

void Auto2() {
    
}

void Skills() {
    Auto1();
    pros::Task taskCup {[&taskCup] {
        while (true) {
            if (clampDistance.get() <= distanceToClampCup) {
                clamp.set_value(true);
                taskCup.suspend();
            }
            pros::delay(20);
        }
    }};
    for (int i = 0; i < 2; i++) {
        chassis.MoveToPosition(-1.25, 53.25, 90, 4);
        chassis.MoveToPosition(-12.25, 53.25, 90, 4);
        lift.move_relative(960 + 270 * i, 127);
        chassis.MoveToPosition(-12.25, 53.25, 180, 4);
        chassis.MoveToPosition(-12.25, 27.25, 180, 4);
        lift.move_relative(-360, 50);
        clamp.retract();
        pros::delay(250);
        lift.move_relative(-600 - 270 * i, 127);
        taskCup.resume();
    }
    taskCup.suspend();
    pros::Task taskPin {[&taskPin] {
        while (true) {
            if (clampDistance.get() <= distanceToClampCup) {
                clamp.set_value(true);
                taskPin.suspend();
            }
            pros::delay(20);
        }
    }};
    chassis.MoveToPosition(-1.25, 53.25, 90, 4);
    chassis.MoveToPosition(-12.25, 53.25, 90, 4);
    lift.move_relative(960, 127);
    chassis.MoveToPosition(-12.25, 53.25, 180, 4);
    chassis.MoveToPosition(-60, 29.25, 180, 4);
    lift.move_relative(-360, 50);
    clamp.retract();
    lift.move_relative(270, 127);
}