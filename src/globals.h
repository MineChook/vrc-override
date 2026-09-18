#pragma once

#include "api.h"
#include "motion/chassis.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"

inline pros::Controller controller1(pros::E_CONTROLLER_MASTER);
inline pros::Controller controller2(pros::E_CONTROLLER_PARTNER);

inline pros::Motor frontLeft(2, pros::MotorGears::blue, pros::MotorEncoderUnits::degrees);
inline pros::Motor frontRight(10, pros::MotorGears::blue, pros::MotorEncoderUnits::degrees);
inline pros::Motor backLeft(6, pros::MotorGears::blue, pros::MotorEncoderUnits::degrees);
inline pros::Motor backRight(4, pros::MotorGears::blue, pros::MotorEncoderUnits::degrees);

inline pros::MotorGroup lift({1, -5}, pros::MotorGear::green, pros::MotorUnits::degrees);
inline pros::MotorGroup intake({11, 12, 18}, pros::MotorGear::blue, pros::MotorUnits::degrees);
inline pros::Distance distance(15);
inline pros::adi::Pneumatics clamp('H', false);

inline pros::Imu imu(7);
inline pros::Rotation leftVerticalTrackingWheel(3);
inline pros::Rotation rightVerticalTrackingWheel(-4);
inline pros::Rotation horizontalTrackingWheel(19);

inline Odometry odometry(0, 0, 0, 3.25, 2);

inline DriveControllerData driveController(0.85, 127, 5, ControllerData<double>(0, 0, 0));
inline ControllerData<Eigen::Vector2d> linearController(10, 0, 12);
inline ControllerData<double> angularController(5, 0, 2);

inline Chassis chassis(frontLeft, frontRight, backLeft, backRight, odometry, driveController, linearController, angularController);