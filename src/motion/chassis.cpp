#include "chassis.h"
#include "../globals.h"
#include "pros/misc.h"
#include <cmath>

void Chassis::Calibrate() {
    imu.reset(true);

    controller1.rumble("- . -");
}

void Chassis::MoveVoltage(int frontLeftVoltage, int frontRightVoltage, int middleLeftVoltage, int middleRightVoltage, int backLeftVoltage, int backRightVoltage) {
    m_frontLeft.move(frontLeftVoltage);
    m_frontRight.move(frontRightVoltage);
    m_middleLeft.move(middleLeftVoltage);
    m_middleRight.move(middleRightVoltage);
    m_backLeft.move(backLeftVoltage);
    m_backRight.move(backRightVoltage);
}

void Chassis::CentricArcade(int forwardSpeed, int strafeSpeed, int turningSpeed, bool fieldCentric) {

    double currentHeading = imu.get_heading();

    if(abs(strafeSpeed) < this->m_driveControllerData.getDeadzone()) strafeSpeed = 0;
    if(abs(forwardSpeed) < this->m_driveControllerData.getDeadzone()) forwardSpeed = 0;
    if(abs(turningSpeed) < this->m_driveControllerData.getDeadzone()) {

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
        this->m_driveControllerData.getTurningControllerData().setError(0);
        this->m_driveControllerData.getTurningControllerData().setIntegral(0);
        this->m_driveControllerData.getTurningControllerData().setDerivative(0);
        this->m_driveControllerData.getTurningControllerData().setLastError(0);
        turningSpeed *= this->m_driveControllerData.getSensitivity();
    }

    if (fieldCentric) {
        int thetaHeading = currentHeading * M_PI / 180;

        int tempForwardSpeed = strafeSpeed * sin(thetaHeading) + forwardSpeed * cos(thetaHeading);
        strafeSpeed = strafeSpeed * cos(thetaHeading) - forwardSpeed * sin(thetaHeading);
        forwardSpeed = tempForwardSpeed;
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

void Chassis::MoveToPosition(double targetX, double targetY, double targetHeading, int timeoutSeconds) {
    this->m_driveControllerData.getTurningControllerData().setError(0);
    this->m_driveControllerData.getTurningControllerData().setIntegral(0);
    this->m_driveControllerData.getTurningControllerData().setDerivative(0);
    this->m_driveControllerData.getTurningControllerData().setLastError(0);

    int timeoutMilliseconds = timeoutSeconds * 1000;

    while (timeoutMilliseconds > 0) {
        
        timeoutMilliseconds -= 20;
        pros::delay(20);
    }
}