#include "autonomous.h"
#include "globals.h"
#include "pros/screen.hpp"

//drivebase length = 15.75, drivebase width = 15, half length = 7.875, half width = 7.5
void Auto1() {
    toggles.move(127);
    lift.move_relative(90, 127);
    pros::delay(250);
    toggles.move(0);
    chassis.MoveToPosition(-16, 27, 0, 2, 0.6);
    lift.move_relative(-90, 127);
    clamp.set_value(false);

}

void Auto2() {
    controller1.print(0, 0, "Auto2");
    chassis.MoveToPosition(24, 24, 0, 100, 0.6);
}

void Skills() {
    chassis.MoveToPosition(0, 24, 0, 100);
}