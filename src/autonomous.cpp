#include "autonomous.h"
#include "globals.h"

void Auto1() {
    intake.move(-127);
    lift.move(127);
    pros::delay(3000);
    chassis.MoveToPosition(36, -24, 0, 0.5);
}

void Auto2() {
    intake.move(-127);
    lift.move(127);
    pros::delay(3000);
    chassis.MoveToPosition(36, -24, 0, 0.5);
}

void Skills() {
    chassis.MoveToPosition(36, -72, 0, 1.5);
}