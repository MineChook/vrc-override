#pragma once

#include "api.h"
#include "motion/chassis.h"
#include "pros/abstract_motor.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"

inline pros::Controller controller1(pros::E_CONTROLLER_MASTER);

inline pros::Motor frontLeft(1, pros::MotorGears::blue, pros::MotorEncoderUnits::degrees);
inline pros::Motor frontRight(4, pros::MotorGears::blue, pros::MotorEncoderUnits::degrees);
inline pros::Motor middleLeft(2, pros::MotorGears::blue, pros::MotorEncoderUnits::degrees);
inline pros::Motor middleRight(5, pros::MotorGears::blue, pros::MotorEncoderUnits::degrees);
inline pros::Motor backLeft(3, pros::MotorGears::blue, pros::MotorEncoderUnits::degrees);
inline pros::Motor backRight(6, pros::MotorGears::blue, pros::MotorEncoderUnits::degrees);

inline pros::MotorGroup lift({11, 12}, pros::MotorGear::green, pros::MotorUnits::degrees);

inline pros::Imu imu(7);
inline pros::Rotation verticalTrackingWheel(8);
inline pros::Rotation horizontalTrackingWheel(9);

inline Odometry odometry(0, 0);

inline DriveControllerData driveController(1, 127, 5, ControllerData<double>(1.2, 0, 0.2));
inline ControllerData<Eigen::Vector2d> linearController(0, 0, 0);
inline ControllerData<double> angularController(0, 0, 0);

inline Chassis chassis(frontLeft, frontRight, middleLeft, middleRight, backLeft, backRight, odometry, driveController, linearController, angularController);