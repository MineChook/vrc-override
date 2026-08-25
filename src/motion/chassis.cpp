#include "chassis.h"
#include "../globals.h"
#include "pros/misc.h"
#include <cmath>
#define EIGEN_DONT_VECTORIZE
#include "Eigen/Dense"

const Eigen::Matrix<double, 4, 3> kinematicsMatrix = (Eigen::Matrix<double, 4, 3>() << // (forward, strafe, turn)
    1, 1, 1, // front left
    -1, 1, 1, // front right
    1, -1, 1, // back left
    -1, -1, 1 // back right
).finished();

void Chassis::Calibrate() {
    imu.reset(true);

    m_odometry.SetPosition(0, 0, 0);

    controller1.rumble("- . -");
}

void Chassis::MoveVoltage(int frontLeftVoltage, int frontRightVoltage, int backLeftVoltage, int backRightVoltage) {
    m_frontLeft.move(frontLeftVoltage);
    m_frontRight.move(frontRightVoltage);
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

    double vy = forwardSpeed;
    double vx = strafeSpeed;

    if (fieldCentric) {
        double thetaHeading = currentHeading * M_PI / 180;

        Eigen::Matrix2d rotation;
        rotation << std::cos(thetaHeading), std::sin(thetaHeading), -std::sin(thetaHeading), std::cos(thetaHeading);

        Eigen::Vector2d fieldInput(vy, vx);
        Eigen::Vector2d robotInput = rotation * fieldInput;

        vx = robotInput.x();
        vy = robotInput.y();
    }

    Eigen::Vector3d input(vy, vx, turningSpeed);
    Eigen::Matrix<double, 4, 1> motorVelocities = kinematicsMatrix * input;

    double maxVal = motorVelocities.cwiseAbs().maxCoeff();
    if (maxVal > 127.0) {
        motorVelocities *= (127.0 / maxVal);
    }

    MoveVoltage(static_cast<int>(motorVelocities(0)), static_cast<int>(motorVelocities(1)), static_cast<int>(motorVelocities(2)), static_cast<int>(motorVelocities(3)));
}

void Chassis::MoveToPosition(double targetX, double targetY, double targetHeading, double timeoutSeconds) {
    this->m_linearControllerData.setError(Eigen::Vector2d(0, 0));
    this->m_linearControllerData.setIntegral(Eigen::Vector2d(0, 0));
    this->m_linearControllerData.setDerivative(Eigen::Vector2d(0, 0));
    this->m_linearControllerData.setLastError(Eigen::Vector2d(0, 0));
    this->m_angularControllerData.setError(0);
    this->m_angularControllerData.setIntegral(0);
    this->m_angularControllerData.setDerivative(0);
    this->m_angularControllerData.setLastError(0);

    int timeoutMilliseconds = timeoutSeconds * 1000;

    while (timeoutMilliseconds > 0) {
        double currentHeading = m_odometry.GetHeading();

        double xError = targetX - m_odometry.GetX();
        double yError = targetY - m_odometry.GetY();
        double headingError = targetHeading - currentHeading;

        while (headingError > M_PI) {
            headingError -= 2 * M_PI;
        }
        while (headingError < -M_PI) {
            headingError += 2 * M_PI;
        }

        Eigen::Matrix2d rotation;
        rotation <<  std::cos(currentHeading), -std::sin(currentHeading), std::sin(currentHeading),  std::cos(currentHeading);

        Eigen::Vector2d globalError(xError, yError);
        Eigen::Vector2d translationError = rotation * globalError;

        double distanceError = std::hypot(translationError.x(), translationError.y());
        if (distanceError < m_linearControllerData.getDistanceTolerance() && std::abs(headingError) < m_angularControllerData.getHeadingTolerance()) {
            MoveVoltage(0, 0, 0, 0);
            return;
        }

        m_linearControllerData.setError(translationError);
        m_linearControllerData.setIntegral(m_linearControllerData.getIntegral() + translationError);
        m_linearControllerData.setDerivative(translationError - m_linearControllerData.getLastError());

        double vx = m_linearControllerData.getKp() * m_linearControllerData.getError().x() + m_linearControllerData.getKi() * m_linearControllerData.getIntegral().x() + m_linearControllerData.getKd() * m_linearControllerData.getDerivative().x();
        double vy = m_linearControllerData.getKp() * m_linearControllerData.getError().y() + m_linearControllerData.getKi() * m_linearControllerData.getIntegral().y() + m_linearControllerData.getKd() * m_linearControllerData.getDerivative().y();

        m_angularControllerData.setError(headingError);
        m_angularControllerData.setIntegral(m_angularControllerData.getIntegral() + headingError);
        m_angularControllerData.setDerivative(headingError - m_angularControllerData.getLastError());

        double turningSpeed = m_angularControllerData.getKp() * headingError + m_angularControllerData.getKi() * m_angularControllerData.getIntegral() + m_angularControllerData.getKd() * m_angularControllerData.getDerivative();

        Eigen::Vector3d velocityInput(vy, vx, turningSpeed);
        Eigen::Matrix<double, 4, 1> motorVelocities = kinematicsMatrix * velocityInput;

        double maxVal = motorVelocities.cwiseAbs().maxCoeff();
        if (maxVal > 127.0) {
            motorVelocities *= (127.0 / maxVal);
        }

        MoveVoltage(static_cast<int>(motorVelocities(0)), static_cast<int>(motorVelocities(1)), static_cast<int>(motorVelocities(2)), static_cast<int>(motorVelocities(3)));

        m_linearControllerData.setLastError(translationError);
        m_angularControllerData.setLastError(headingError);

        timeoutMilliseconds -= 20;
        pros::delay(20);
    }

    MoveVoltage(0, 0, 0, 0);
}