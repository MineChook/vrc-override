#include "clawPivotController.h"
#include "main.h"
#include "../globals.h"
#include "pros/misc.h"

void ClawPivotController::StartLoop() {
    // Start a new task for the PID loop
    pros::Task loopTask([this]() {
        while (true) {
            double position = this->m_motor.get_position();

            if (controller2.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) > this->m_controllerData.getDeadzone()) {
                this->m_controllerData.setTargetHeading(this->m_controllerData.getTargetHeading() + 1);
		    }
            else if (controller2.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) < -this->m_controllerData.getDeadzone()) {
                this->m_controllerData.setTargetHeading(this->m_controllerData.getTargetHeading() - 1);
            }

            this->m_controllerData.getTurningControllerData().setError(this->m_controllerData.getTargetHeading() - position);
            
            if (this->m_controllerData.getTurningControllerData().getError() > 180) {
                this->m_controllerData.getTurningControllerData().setError(this->m_controllerData.getTurningControllerData().getError() - 360);
            } else if (this->m_controllerData.getTurningControllerData().getError() < -180) {
                this->m_controllerData.getTurningControllerData().setError(this->m_controllerData.getTurningControllerData().getError() + 360);
            }

            this->m_controllerData.getTurningControllerData().setIntegral(this->m_controllerData.getTurningControllerData().getIntegral() + this->m_controllerData.getTurningControllerData().getError());

            this->m_controllerData.getTurningControllerData().setDerivative(this->m_controllerData.getTurningControllerData().getError() - this->m_controllerData.getTurningControllerData().getLastError());
            this->m_controllerData.getTurningControllerData().setLastError(this->m_controllerData.getTurningControllerData().getError());

            double turningSpeed = this->m_controllerData.getTurningControllerData().getKp() * this->m_controllerData.getTurningControllerData().getError() + this->m_controllerData.getTurningControllerData().getKi() * this->m_controllerData.getTurningControllerData().getIntegral() + this->m_controllerData.getTurningControllerData().getKd() * this->m_controllerData.getTurningControllerData().getDerivative();

            if (turningSpeed > 127) {
                turningSpeed = 127;
            }

            this->m_motor.move(turningSpeed);

            pros::delay(20);
        }
    }, "TogglePivot PID Loop");
}