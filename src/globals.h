#pragma once

#include "api.h"
#include "motion/chassis.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"

inline pros::Controller controller1(pros::E_CONTROLLER_MASTER);

inline pros::Motor frontLeft(2, pros::MotorGears::blue, pros::MotorEncoderUnits::degrees);
inline pros::Motor frontRight(10, pros::MotorGears::blue, pros::MotorEncoderUnits::degrees);
inline pros::Motor backLeft(6, pros::MotorGears::blue, pros::MotorEncoderUnits::degrees);
inline pros::Motor backRight(4, pros::MotorGears::blue, pros::MotorEncoderUnits::degrees);

inline pros::MotorGroup lift({1, -5}, pros::MotorGear::green, pros::MotorUnits::degrees);
inline pros::MotorGroup intake({11, 12, 13}, pros::MotorGear::blue, pros::MotorUnits::degrees);
inline pros::adi::Pneumatics scoringPiston('H', false);

inline pros::Imu imu(9);
inline pros::Rotation verticalTrackingWheel(14);
inline pros::Rotation horizontalTrackingWheel(15);

inline Odometry odometry(0, 0);

inline DriveControllerData driveController(1, 127, 5, ControllerData<double>(0, 0, 0));
inline ControllerData<Eigen::Vector2d> linearController(0, 0, 0);
inline ControllerData<double> angularController(0, 0, 0);

inline Chassis chassis(frontLeft, frontRight, backLeft, backRight, odometry, driveController, linearController, angularController);