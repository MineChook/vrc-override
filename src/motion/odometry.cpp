#include "odometry.h"
#include "../globals.h"
#include "pros/llemu.hpp"
#include <charconv>
#include <cmath>
#include <string>
#define EIGEN_DONT_VECTORIZE
#include "Eigen/Core"

double Odometry::DegreesToDistance(double degrees, uint8_t wheelType) {
    double realDegrees = degrees / 100.0; // Convert centidegrees to degrees

    if (wheelType == 0) {
        return (realDegrees / 360.0) * (m_verticalWheelDiameter * M_PI);
    } else {
        return (realDegrees / 360.0) * (m_horizontalWheelDiameter * M_PI);
    }
}

float Round(float var)
{
    float value = (int)(var * 100 + .5);
    return (float)value / 100;
}

Eigen::Vector2d Odometry::GetPosition() {
    return m_position;
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

            if (std::isnan(leftVerticalTrackingWheel.get_position()) || std::isinf(leftVerticalTrackingWheel.get_position()) || std::isnan(rightVerticalTrackingWheel.get_position()) || std::isinf(rightVerticalTrackingWheel.get_position()) || std::isnan(horizontalTrackingWheel.get_position()) || std::isinf(horizontalTrackingWheel.get_position())) {
                controller2.set_text(1, 0, "Error: Invalid sensor reading");
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

            odometry->SetPosition(odometry->m_position[0] + globalTranslation.x(), odometry->m_position[1] + globalTranslation.y(), currentHeadingRadians);

            odometry->m_lastHeading = currentHeadingRadians;

            if (std::isnan(odometry->m_position[0]) || std::isinf(odometry->m_position[0])) {
                odometry->m_position[0] = 0;
            }
            if (std::isnan(odometry->m_position[1]) || std::isinf(odometry->m_position[1])) {
                odometry->m_position[1] = 0;
            }

            controller2.set_text(0, 0, "X: " + std::to_string(Round(odometry->m_position[0])) + " Y: " + std::to_string(Round(odometry->m_position[1])) + " H: " + std::to_string(Round(odometry->m_heading * 180 / M_PI)));
            pros::delay(20); // Delay for 20 milliseconds
        }
    }, this);
}

void Odometry::StopUpdating() {
    m_stopTask = true;
}

void Odometry::SetPosition(double x, double y, double heading) {
    m_position << x, y;
    m_heading = heading;
    m_lastHeading = heading;
}