#include "autonomous.h"
#include "globals.h"
#include "pros/screen.hpp"

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
    chassis.MoveToPosition(0, 24, 0, 100);
}