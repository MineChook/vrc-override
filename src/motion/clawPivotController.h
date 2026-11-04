#pragma once

#include "main.h"
#include "controller.hpp"
#include "pros/motors.hpp"
#include <sys/types.h>

class ClawPivotController {
    private:
        pros::Motor& m_motor;
        DriveControllerData& m_controllerData;

    public:
        ClawPivotController(pros::Motor& motor, DriveControllerData& controllerData) : m_motor(motor), m_controllerData(controllerData){};

        pros::Motor& getMotor() {
            return m_motor;
        }

        DriveControllerData& getControllerData() {
            return m_controllerData;
        }

        void StartLoop();
};