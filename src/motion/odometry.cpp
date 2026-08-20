#include "odometry.h"
#include "../globals.h"
#define EIGEN_DONT_VECTORIZE
#include "Eigen/Core"

double degreesToDistance(double degrees) {
    return (degrees / 360.0) * (2.0 * M_PI);
}

double Odometry::getX() {
    return m_x;
}

double Odometry::getY() {
    return m_y;
}

double Odometry::getHeading() {
    return m_heading;
}

void Odometry::startUpdating() {
    // Start a new task to update the odometry values
    pros::Task odometryTask([](void* param) {
        Odometry* odometry = static_cast<Odometry*>(param);
        while (true) {
            if (odometry->m_stopTask) {
                break; // Exit the loop if the task is stopped
            }

            double currentVerticalDegrees = verticalTrackingWheel.get_position();
            double currentHorizontalDegrees = horizontalTrackingWheel.get_position();
            double currentHeadingRadians = imu.get_heading() * M_PI / 180.0;

            double verticalRaw = degreesToDistance(currentVerticalDegrees - odometry->m_lastVerticalDegrees);
            double horizontalRaw = degreesToDistance(currentHorizontalDegrees - odometry->m_lastHorizontalDegrees);
            double headingRaw = currentHeadingRadians - odometry->m_lastHeading;

            if (headingRaw > M_PI) {
                headingRaw -= 2 * M_PI;
            } else if (headingRaw < -M_PI) {
                headingRaw += 2 * M_PI;
            }

            double deltaYLocal = verticalRaw - odometry->m_verticalTrackingWheelOffset * headingRaw;
            double deltaXLocal = horizontalRaw - odometry->m_horizontalTrackingWheelOffset * headingRaw;

            double averageHeading = odometry->m_lastHeading + headingRaw / 2.0;

            Eigen::Matrix2d globalRotation;
            globalRotation << std::cos(averageHeading), -std::sin(averageHeading), std::sin(averageHeading), std::cos(averageHeading);
            Eigen::Vector2d localTranslation(deltaXLocal, deltaYLocal);
            Eigen::Vector2d globalTranslation = globalRotation * localTranslation;

            odometry->m_x += globalTranslation.x();
            odometry->m_y += globalTranslation.y();
            odometry->m_heading = currentHeadingRadians;

            odometry->m_lastVerticalDegrees = currentVerticalDegrees;
            odometry->m_lastHorizontalDegrees = currentHorizontalDegrees;
            odometry->m_lastHeading = currentHeadingRadians;

            pros::delay(20); // Delay for 20 milliseconds
        }
    }, this);
}

void Odometry::stopUpdating() {
    m_stopTask = true;
}

void Odometry::setPosition(double x, double y, double heading) {
    m_x = x;
    m_y = y;
    m_heading = heading;
}