#pragma once

#include "main.h"
#include "controller.hpp"
#include "pros/motors.hpp"
#include <sys/types.h>

class ClawPivotController {
    private:
        pros::Motor& m_motor;
        bool m_controlled = false;
\
        u_int8_t tick = 0;
        DriveControllerData& m_controllerData;

    public:
        ClawPivotController(pros::Motor& motor, DriveControllerData& controllerData) : m_motor(motor), m_controllerData(controllerData){};

        void StartLoop();
};