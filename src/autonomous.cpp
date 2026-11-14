#include "autonomous.h"
#include "globals.h"
#include "pros/screen.hpp"

//drivebase length = 15.75, drivebase width = 15, half length = 7.875, half width = 7.5
void Auto1() {
    chassis.MoveToPosition(1.5, 0, 45, 2, 0.6);
    toggles.move_absolute(180, 127);
    pros::delay(250);
}

void Auto2() {
    chassis.MoveToPosition(1.5, 0, 45, 2, 0.6);
    toggles.move_absolute(90, 127);
    pros::delay(250);
    // controller1.print(0, 0, "Auto2");
    // chassis.MoveToPosition(24, 24, 0, 100, 0.6);
}

void Skills() {
    chassis.MoveToPosition(0, 24, 0, 100);
}