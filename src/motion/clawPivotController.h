#pragma once

#include "main.h"
#include "controller.hpp"
#include "pros/motors.hpp"

class ClawPivotController {
    private:
        pros::Motor& m_motor;
        bool m_controlled = false;

        DriveControllerData& m_controllerData;

    public:
        ClawPivotController(pros::Motor& motor, DriveControllerData& controllerData) : m_motor(motor), m_controllerData(controllerData){};

        void StartLoop();
};