#include "odometry.h"
#include "../globals.h"
#include "pros/llemu.hpp"
#include <charconv>
#include <cmath>
#include <string>
#define EIGEN_DONT_VECTORIZE
#include "Eigen/Core"

double Odometry::DegreesToDistance(double degrees, uint8_t wheelType) {
    if (wheelType == 0) {
        return (degrees / 360.0) * (m_verticalWheelDiameter * M_PI);
    } else {
        return (degrees / 360.0) * (m_horizontalWheelDiameter * M_PI);
    }
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
        
            odometry->m_lastLeftVerticalDegrees = leftVerticalTrackingWheel.get_position();
            odometry->m_lastRightVerticalDegrees = rightVerticalTrackingWheel.get_position();
            odometry->m_lastHorizontalDegrees = horizontalTrackingWheel.get_position();

        while (true) {
            if (odometry->m_stopTask) {
                break; // Exit the loop if the task is stopped
            }

            if (std::isnan(frontLeft.get_position()) || std::isinf(frontLeft.get_position()) || std::isnan(frontRight.get_position()) || std::isinf(frontRight.get_position()) || std::isnan(backLeft.get_position()) || std::isinf(backLeft.get_position()) || std::isnan(backRight.get_position()) || std::isinf(backRight.get_position())) {
                pros::delay(20);
                continue;
            }

            double headingRaw = 0;
            double deltaXLocal = 0;
            double deltaYLocal = 0;

            double currentLeftVerticalDegrees = leftVerticalTrackingWheel.get_position();
            double currentRightVerticalDegrees = rightVerticalTrackingWheel.get_position();
            double currentHorizontalDegrees = horizontalTrackingWheel.get_position();

            double leftVerticalRaw = odometry->DegreesToDistance(currentLeftVerticalDegrees - odometry->m_lastLeftVerticalDegrees, 0);
            double rightVerticalRaw = odometry->DegreesToDistance(currentRightVerticalDegrees - odometry->m_lastRightVerticalDegrees, 0);
            double horizontalRaw = odometry->DegreesToDistance(currentHorizontalDegrees - odometry->m_lastHorizontalDegrees, 1);

            double trackingWidth = std::abs(odometry->m_verticalRightTrackingWheelOffset) + std::abs(odometry->m_verticalLeftTrackingWheelOffset);
            headingRaw = (rightVerticalRaw - leftVerticalRaw) / trackingWidth;

            deltaYLocal = (leftVerticalRaw + rightVerticalRaw) / 2.0;
            deltaXLocal = horizontalRaw - headingRaw * odometry->m_horizontalTrackingWheelOffset;

            odometry->m_lastLeftVerticalDegrees = currentLeftVerticalDegrees;
            odometry->m_lastRightVerticalDegrees = currentRightVerticalDegrees;
            odometry->m_lastHorizontalDegrees = currentHorizontalDegrees;

            double currentHeadingRadians = odometry->m_lastHeading + headingRaw;

            while (currentHeadingRadians > M_PI) currentHeadingRadians -= 2 * M_PI;
            while (currentHeadingRadians < -M_PI) currentHeadingRadians += 2 * M_PI;

            double averageHeading = odometry->m_lastHeading + (headingRaw / 2.0);

            Eigen::Matrix2d globalRotation;
            globalRotation << std::cos(averageHeading), -std::sin(averageHeading), std::sin(averageHeading), std::cos(averageHeading);
            Eigen::Vector2d localTranslation(deltaXLocal, deltaYLocal);
            Eigen::Vector2d globalTranslation = globalRotation * localTranslation;

            odometry->SetPosition(odometry->GetX() + globalTranslation.x(), odometry->GetY() + globalTranslation.y(), currentHeadingRadians);

            odometry->m_lastHeading = currentHeadingRadians;

            if (std::isnan(odometry->m_x) || std::isinf(odometry->m_x)) {
                odometry->m_x = 0;
            }
            else if (std::isnan(odometry->m_y) || std::isinf(odometry->m_y)) {
                odometry->m_y = 0;
            }
            std::cout << odometry->GetX() << ", " << odometry->GetY() << ", " << odometry->GetHeading() << std::endl;

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