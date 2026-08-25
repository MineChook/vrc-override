#include "odometry.h"
#include "../globals.h"
#include "pros/llemu.hpp"
#include <charconv>
#include <cmath>
#include <string>
#define EIGEN_DONT_VECTORIZE
#include "Eigen/Core"

double degreesToDistance(double degrees) {
    return (degrees / 360.0) * (chassis.GetOdometry().GetWheelDiameter() / 2 * M_PI);
}

double Odometry::GetX() {
    return m_x;
}

double Odometry::GetY() {
    return m_y;
}

double Odometry::GetHeading() {
    return m_heading;
}

double Odometry::GetWheelDiameter() {
    return 3.25;
}

void Odometry::StartUpdating() {
    // Start a new task to update the odometry values
    pros::Task odometryTask([](void* param) {
        Odometry* odometry = static_cast<Odometry*>(param);
        while (imu.is_calibrating()) {
            pros::delay(20);
        }

        if (odometry->m_type == OdometryType::TRACKING_WHEELS) {
            odometry->m_lastVerticalDegrees = verticalTrackingWheel.get_position();
            odometry->m_lastHorizontalDegrees = horizontalTrackingWheel.get_position();
        } else if (odometry->m_type == OdometryType::X_DRIVE) {
            odometry->m_lastForwardLeftDegrees = chassis.GetFrontLeftMotor().get_position();
            odometry->m_lastForwardRightDegrees = chassis.GetFrontRightMotor().get_position();
            odometry->m_lastBackLeftDegrees = chassis.GetBackLeftMotor().get_position();
            odometry->m_lastBackRightDegrees = chassis.GetBackRightMotor().get_position();
        }

        while (true) {
            if (odometry->m_stopTask) {
                break; // Exit the loop if the task is stopped
            }

            if (std::isnan(frontLeft.get_position()) || std::isinf(frontLeft.get_position()) || std::isnan(frontRight.get_position()) || std::isinf(frontRight.get_position()) || std::isnan(backLeft.get_position()) || std::isinf(backLeft.get_position()) || std::isnan(backRight.get_position()) || std::isinf(backRight.get_position())) {
                pros::delay(20);
                continue;
            }

            double currentHeadingRadians = imu.get_heading() * M_PI / 180.0;
            double headingRaw = currentHeadingRadians - odometry->m_lastHeading;

            if (headingRaw > M_PI) {
                headingRaw -= 2 * M_PI;
            } else if (headingRaw < -M_PI) {
                headingRaw += 2 * M_PI;
            }

            double deltaXLocal = 0;
            double deltaYLocal = 0;

            if (odometry->m_type == OdometryType::TRACKING_WHEELS) {

                double currentVerticalDegrees = verticalTrackingWheel.get_position();
                double currentHorizontalDegrees = horizontalTrackingWheel.get_position();

                double verticalRaw = degreesToDistance(currentVerticalDegrees - odometry->m_lastVerticalDegrees);
                double horizontalRaw = degreesToDistance(currentHorizontalDegrees - odometry->m_lastHorizontalDegrees);

                deltaYLocal = verticalRaw - odometry->m_verticalTrackingWheelOffset * headingRaw;
                deltaXLocal = horizontalRaw - odometry->m_horizontalTrackingWheelOffset * headingRaw;

                odometry->m_lastVerticalDegrees = currentVerticalDegrees;
                odometry->m_lastHorizontalDegrees = currentHorizontalDegrees;
            }
            else if (odometry->m_type == OdometryType::X_DRIVE) {
                double leftFrontDegrees = chassis.GetFrontLeftMotor().get_position();
                double rightFrontDegrees = chassis.GetFrontRightMotor().get_position();
                double leftBackDegrees = chassis.GetBackLeftMotor().get_position();
                double rightBackDegrees = chassis.GetBackRightMotor().get_position();

                double leftFrontDistance = degreesToDistance(leftFrontDegrees - odometry->m_lastForwardLeftDegrees);
                double rightFrontDistance = degreesToDistance(rightFrontDegrees - odometry->m_lastForwardRightDegrees);
                double leftBackDistance = degreesToDistance(leftBackDegrees - odometry->m_lastBackLeftDegrees);
                double rightBackDistance = degreesToDistance(rightBackDegrees - odometry->m_lastBackRightDegrees);

                deltaYLocal = (leftFrontDistance - rightFrontDistance + leftBackDistance - rightBackDistance) / 4.0 * std::sqrt(2.0);
                deltaXLocal = (leftFrontDistance + rightFrontDistance - leftBackDistance - rightBackDistance) / 4.0 * std::sqrt(2.0);

                odometry->m_lastForwardLeftDegrees = leftFrontDegrees;
                odometry->m_lastForwardRightDegrees = rightFrontDegrees;
                odometry->m_lastBackLeftDegrees = leftBackDegrees;
                odometry->m_lastBackRightDegrees = rightBackDegrees;
            }

            double averageHeading = odometry->m_lastHeading + headingRaw / 2.0;

            Eigen::Matrix2d globalRotation;
            globalRotation << std::cos(averageHeading), -std::sin(averageHeading), std::sin(averageHeading), std::cos(averageHeading);
            Eigen::Vector2d localTranslation(deltaXLocal, deltaYLocal);
            Eigen::Vector2d globalTranslation = globalRotation * localTranslation;

            odometry->SetPosition(odometry->GetX() + globalTranslation.x(), odometry->GetY() + globalTranslation.y(), currentHeadingRadians);
            if (std::isnan(odometry->m_x) || std::isinf(odometry->m_x)) {
                odometry->m_x = 0;
            }
            else if (std::isnan(odometry->m_y) || std::isinf(odometry->m_y)) {
                odometry->m_y = 0;
            }
            std::cout << odometry->GetX() << ", " << odometry->GetY() << ", " << odometry->GetHeading() << std::endl;
            odometry->m_lastHeading = currentHeadingRadians;

            pros::delay(20); // Delay for 20 milliseconds
        }
    }, this);
}

void Odometry::StopUpdating() {
    m_stopTask = true;
}

void Odometry::SetPosition(double x, double y, double heading) {
    m_x = x;
    m_y = y;
    m_heading = heading;
}