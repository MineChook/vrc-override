#include "autonomous.h"
#include "globals.h"
#include "pros/screen.hpp"

void Auto1() {
    intake.move_relative(270, 127);
}

void Auto2() {
    controller1.print(1, 0, "test2");
    chassis.MoveToPosition(0, 24, 0, 100);
}

void Skills() {
    controller1.print(1, 0, "test");
    chassis.MoveToPosition(0, 24, 0, 100);
}