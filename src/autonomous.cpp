#include "autonomous.h"
#include "globals.h"
#include "main.h"
#include "motion/chassis.h"
#include "motion/clawPivotController.h"
#include "pros/screen.hpp"
#include "selector.h"

//drivebase length = 15.75, drivebase width = 15, half length = 7.875, half width = 7.5
void Auto1() {
    /*toggles.move(127);
    chassis.GetOdometry().SetPosition(0, 8, 45);
    chassis.MoveToPosition(0, 0, 0, 2.0);
    pros::delay(10000); // temp
    lift.move_absolute(90, 127);
    pros::delay(250);
    toggles.move(0);
    chassis.MoveToPosition(-16, 19, 0, 2, 0.6);
    lift.move_absolute(0, 127);
    pros::delay(250);
    clamp.set_value(false);
    pros::delay(250);
    chassis.MoveToPosition(-16, 15, 0, 1);
    pros::delay(250);
    chassis.MoveToPosition(-16, 15, -73, 1);
    pros::delay(250);
    chassis.MoveToPosition(-28.5, 29.25, -90, 2.25, -0.6, 90);
    pros::delay(250);
    chassis.MoveToPosition(-37.5, 29.25, -90, 1, 0, 40);
    clamp.set_value(true);
    lift.move_absolute(360, 127);
    chassis.MoveToPosition(-43, 29.25, 90, 1.75);
    pros::delay(250);
    chassis.MoveToPosition(-25, 29.25, 90, 1.75);
    pros::delay(250);
    lift.move_absolute(180, 80);
    pros::delay(250);
    clamp.set_value(false);
    */
    
}

void Auto2() {
    controller1.print(0, 0, "Auto2");
    chassis.MoveToPosition(24, 24, 0, 100, 0.6);
}

void Skills() {
    lift.move_relative(90, 100);
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
    lift.move_relative(-90, 60);
    clawPivotController.getControllerData().setTargetHeading(clawPivotController.getMotor().get_position() + 15);
    pros::delay(250);
    clamp.retract();
    pros::delay(250);
    chassis.GetOdometry().SetPosition(-14, 8.8, chassis.GetOdometry().GetHeading());
    chassis.MoveToPosition(-37.2, 0, 0, 1.25, 0.8);
    pros::delay(250);
    chassis.MoveToPosition(-37.2, 6.2, 0, 1.25, 0, 64);
    pros::delay(250);
    clamp.extend();
    lift.move_relative(270, 80);
    chassis.MoveToPosition(-23, 18.4, 90, 1.5);
    lift.move_relative(-180, 80);
    clamp.retract();
}