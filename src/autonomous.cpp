#include "autonomous.h"
#include "globals.h"
#include "pros/screen.hpp"

void Auto1() {
    intake.move_relative(270, 127);
}

void Auto2() {
    intake.move(-127);
    lift.move(127);
    pros::delay(3000);
    chassis.MoveToPosition(36, -24, 0, 0.5);
}

void Skills() {
    controller1.set_text(1, 0, "skills ran i guess");
    chassis.MoveToPosition(0, 0, 90, 100);
}