
#pragma once

#include "api.h"
#include "controller.hpp"

enum MotorType {
    FrontLeft = 0,
    FrontRight,
    MiddleLeft,
    MiddleRight,
    BackLeft,
    BackRight,
};

/**
 * @brief The chassis class is responsible for controlling the robot's drivetrain. It contains methods for moving the robot in different ways, such as arcade and curvature drive, both in robot-centric and field-centric modes. It will also contain the PID code for autonomous motion.
 */
class Chassis {
private:
    pros::Motor m_frontLeft;
    pros::Motor m_frontRight;
    pros::Motor m_middleLeft;
    pros::Motor m_middleRight;
    pros::Motor m_backLeft;
    pros::Motor m_backRight;

    DriveControllerData m_driveControllerData;
    ControllerData m_linearControllerData;
    ControllerData m_angularControllerData;

public:
    Chassis(pros::Motor& frontLeft, pros::Motor& frontRight, pros::Motor& middleLeft, pros::Motor& middleRight, pros::Motor& backLeft, pros::Motor& backRight, const DriveControllerData& driveControllerData, const ControllerData& linearControllerData, const ControllerData& angularControllerData) : m_frontLeft(std::move(frontLeft)), m_frontRight(std::move(frontRight)), m_middleLeft(std::move(middleLeft)), m_middleRight(std::move(middleRight)), m_backLeft(std::move(backLeft)), m_backRight(std::move(backRight)), m_driveControllerData(std::move(driveControllerData)), m_linearControllerData(std::move(linearControllerData)), m_angularControllerData(std::move(angularControllerData)) {
        
    }

    /**
     * @brief Sets the voltage and moves the robot for each motor in the chassis. Used in opcontrol and autonomous.
     * 
     * @param frontLeftVoltage The voltage to set for the front left motor.
     * @param frontRightVoltage The voltage to set for the front right motor.
     * @param middleLeftVoltage The voltage to set for the middle left motor.
     * @param middleRightVoltage The voltage to set for the middle right motor.
     * @param backLeftVoltage The voltage to set for the back left motor.
     * @param backRightVoltage The voltage to set for the back right motor.
     */
    void MoveVoltage(int frontLeftVoltage, int frontRightVoltage, int middleLeftVoltage, int middleRightVoltage, int backLeftVoltage, int backRightVoltage);

    /**
     * @brief Robot-centric arcade drive. The robot moves relative to the direction it is facing. It does not use curvature drive.
     * 
     * @param forwardSpeed The speed to move forward. Positive values move the robot forward, negative values move the robot backward.
     * @param strafeSpeed The speed to move sideways. Positive values move the robot to the right, negative values move the robot to the left.
     * @param turningSpeed The speed to turn. Positive values turn the robot clockwise, negative values turn the robot counterclockwise.
     */
    void RobotCentricArcade(int forwardSpeed, int strafeSpeed, int turningSpeed);
    void FieldCentricArcade(uint32_t forwardSpeed, uint32_t strafeSpeed, uint32_t turningSpeed);

    void RobotCentricCurvature(int forwardSpeed, int strafeSpeed, int turningSpeed);
    void FieldCentricCurvature(int forwardSpeed, int strafeSpeed, int turningSpeed);
};