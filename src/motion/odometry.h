#pragma once

#include <cstdint>

class Odometry {
private:
    double m_x = 0;
    double m_y = 0;
    double m_heading = 0;
    double m_lastLeftVerticalDegrees = 0;
    double m_lastRightVerticalDegrees = 0;
    double m_lastHorizontalDegrees = 0;
    double m_lastHeading = 0;
    double m_verticalLeftTrackingWheelOffset = 0;
    double m_verticalRightTrackingWheelOffset = 0;
    double m_horizontalTrackingWheelOffset = 0;
    double m_verticalWheelDiameter = 3.25;
    double m_horizontalWheelDiameter = 3.25;
    bool m_stopTask = false;

    double m_lastForwardLeftDegrees = 0;
    double m_lastForwardRightDegrees = 0;
    double m_lastBackLeftDegrees = 0;
    double m_lastBackRightDegrees = 0;
public:
    Odometry(double verticalLeftTrackingWheelOffset = 0, double verticalRightTrackingWheelOffset = 0, double horizontalTrackingWheelOffset = 0, double verticalWheelDiameter = 3.25, double horizontalWheelDiameter = 3.25) : m_verticalLeftTrackingWheelOffset(verticalLeftTrackingWheelOffset), m_verticalRightTrackingWheelOffset(verticalRightTrackingWheelOffset), m_horizontalTrackingWheelOffset(horizontalTrackingWheelOffset), m_verticalWheelDiameter(verticalWheelDiameter), m_horizontalWheelDiameter(horizontalWheelDiameter) {};
    void StartUpdating();
    void StopUpdating();
    double GetX();
    double GetY();
    double GetHeading();
    double DegreesToDistance(double degrees, uint8_t wheelType);
    void SetPosition(double x, double y, double heading);
};