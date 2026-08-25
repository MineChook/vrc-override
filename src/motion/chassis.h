#pragma once

#define EIGEN_DONT_VECTORIZE
#include "Eigen/Core"
#include "api.h"
#include "controller.hpp"
#include "odometry.h"

/**
 * @brief The chassis class is responsible for controlling the robot's drivetrain.
 */
class Chassis {
private:
    pros::Motor& m_frontLeft;
    pros::Motor& m_frontRight;
    pros::Motor& m_backLeft;
    pros::Motor& m_backRight;
    Odometry& m_odometry;

    DriveControllerData m_driveControllerData;
    ControllerData<Eigen::Vector2d> m_linearControllerData;
    ControllerData<double> m_angularControllerData;

public:
    Chassis(pros::Motor& frontLeft, 
            pros::Motor& frontRight, 
            pros::Motor& backLeft, 
            pros::Motor& backRight, 
            Odometry& odometry, 
            const DriveControllerData& driveControllerData, 
            const ControllerData<Eigen::Vector2d>& linearControllerData, 
            const ControllerData<double>& angularControllerData) 
        : m_frontLeft(frontLeft), 
          m_frontRight(frontRight), 
          m_backLeft(backLeft), 
          m_backRight(backRight), 
          m_odometry(odometry), 
          m_driveControllerData(driveControllerData), 
          m_linearControllerData(linearControllerData), 
          m_angularControllerData(angularControllerData) {
    }

    pros::Motor& GetFrontLeftMotor() {
        return m_frontLeft;
    }

    pros::Motor& GetFrontRightMotor() {
        return m_frontRight;
    }

    pros::Motor& GetBackLeftMotor() {
        return m_backLeft;
    }

    pros::Motor& GetBackRightMotor() {
        return m_backRight;
    }

    void Calibrate();

    /**
     * @brief Sets the voltage and moves the robot for each motor in the chassis.
     */
    void MoveVoltage(int frontLeftVoltage, int frontRightVoltage, int backLeftVoltage, int backRightVoltage);

    /**
     * @brief Arcade drive that moves depending on if it is field centric or not.
     */
    void CentricArcade(int forwardSpeed, int strafeSpeed, int turningSpeed, bool fieldCentric = false);

    void MoveToPosition(double targetX, double targetY, double targetHeading, double timeoutSeconds);

    Odometry& GetOdometry() {
        return m_odometry;
    }
}; 