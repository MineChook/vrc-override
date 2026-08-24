#include "odometry.h"
#include "../globals.h"
#include <cmath>
#define EIGEN_DONT_VECTORIZE
#include "Eigen/Core"

double degreesToDistance(double degrees) {
    return (degrees / 360.0) * (2.0 * M_PI);
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

void Odometry::StartUpdating() {
    // Start a new task to update the odometry values
    pros::Task odometryTask([](void* param) {
        Odometry* odometry = static_cast<Odometry*>(param);
        while (true) {
            if (odometry->m_stopTask) {
                break; // Exit the loop if the task is stopped
            }
            double currentHeadingRadians = imu.get_heading() * M_PI / 180.0;
            double headingRaw = currentHeadingRadians - currentHeadingRadians - odometry->m_lastHeading;

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

            odometry->SetPosition(globalTranslation.x(), globalTranslation.y(), currentHeadingRadians);
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