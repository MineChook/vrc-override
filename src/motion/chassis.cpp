#include "chassis.h"
#include "../globals.h"
#include "pros/misc.h"

void Chassis::MoveVoltage(int frontLeftVoltage, int frontRightVoltage, int middleLeftVoltage, int middleRightVoltage, int backLeftVoltage, int backRightVoltage) {
    m_frontLeft.move(frontLeftVoltage);
    m_frontRight.move(frontRightVoltage);
    m_middleLeft.move(middleLeftVoltage);
    m_middleRight.move(middleRightVoltage);
    m_backLeft.move(backLeftVoltage);
    m_backRight.move(backRightVoltage);
}

void Chassis::RobotCentricArcade(int forwardSpeed, int strafeSpeed, int turningSpeed) {

    double currentHeading = imu.get_heading();

    if(abs(strafeSpeed) < this->m_driveControllerData.getDeadzone()) strafeSpeed = 0;
    if(abs(forwardSpeed) < this->m_driveControllerData.getDeadzone()) forwardSpeed = 0;
    if(abs(turningSpeed) < this->m_driveControllerData.getDeadzone()) {
        turningSpeed = 0;

        this->m_driveControllerData.getTurningControllerData().setError(this->m_driveControllerData.getTargetHeading() - currentHeading);

        if (this->m_driveControllerData.getTurningControllerData().getError() > 180) {
            this->m_driveControllerData.getTurningControllerData().setError(this->m_driveControllerData.getTurningControllerData().getError() - 360);
        } else if (this->m_driveControllerData.getTurningControllerData().getError() < -180) {
            this->m_driveControllerData.getTurningControllerData().setError(this->m_driveControllerData.getTurningControllerData().getError() + 360);
        }

        this->m_driveControllerData.getTurningControllerData().setIntegral(this->m_driveControllerData.getTurningControllerData().getIntegral() + this->m_driveControllerData.getTurningControllerData().getError());

        this->m_driveControllerData.getTurningControllerData().setDerivative(this->m_driveControllerData.getTurningControllerData().getError() - this->m_driveControllerData.getTurningControllerData().getLastError());
        this->m_driveControllerData.getTurningControllerData().setLastError(this->m_driveControllerData.getTurningControllerData().getError());

        turningSpeed = this->m_driveControllerData.getTurningControllerData().getKp() * this->m_driveControllerData.getTurningControllerData().getError() + this->m_driveControllerData.getTurningControllerData().getKi() * this->m_driveControllerData.getTurningControllerData().getIntegral() + this->m_driveControllerData.getTurningControllerData().getKd() * this->m_driveControllerData.getTurningControllerData().getDerivative();
    }
    else {
        this->m_driveControllerData.setTargetHeading(currentHeading);
        turningSpeed *= this->m_driveControllerData.getSensitivity();
    }

    int frontLeft = forwardSpeed + strafeSpeed + turningSpeed;
    int frontRight = -forwardSpeed + strafeSpeed + turningSpeed;
    int middleLeft = forwardSpeed + turningSpeed;
    int middleRight = -forwardSpeed + turningSpeed;
    int backLeft = forwardSpeed - strafeSpeed + turningSpeed;
    int backRight = -forwardSpeed - strafeSpeed + turningSpeed;

    int maxValue = std::max({abs(frontLeft), abs(frontRight), abs(middleLeft), abs(middleRight), abs(backLeft), abs(backRight)});

    if (maxValue > 127) {
        frontLeft = frontLeft * 127 / maxValue;
        frontRight = frontRight * 127 / maxValue;
        middleLeft = middleLeft * 127 / maxValue;
        middleRight = middleRight * 127 / maxValue;
        backLeft = backLeft * 127 / maxValue;
        backRight = backRight * 127 / maxValue;
    }

    MoveVoltage(frontLeft, frontRight, middleLeft, middleRight, backLeft, backRight);
}


void Chassis::FieldCentricArcade(uint32_t forwardSpeed, uint32_t strafeSpeed, uint32_t turningSpeed) {
    
}

void Chassis::RobotCentricCurvature(int forwardSpeed, int strafeSpeed, int turningSpeed) {

}