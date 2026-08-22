#pragma once

class Odometry {
private:
    double m_x = 0;
    double m_y = 0;
    double m_heading = 0;
    double m_lastVerticalDegrees = 0;
    double m_lastHorizontalDegrees = 0;
    double m_lastHeading = 0;
    double m_verticalTrackingWheelOffset = 0;
    double m_horizontalTrackingWheelOffset = 0;
    bool m_stopTask = false;
public:
    Odometry(double verticalTrackingWheelOffset, double horizontalTrackingWheelOffset) : m_verticalTrackingWheelOffset(verticalTrackingWheelOffset), m_horizontalTrackingWheelOffset(horizontalTrackingWheelOffset) {};
    void StartUpdating();
    void StopUpdating();
    double GetX();
    double GetY();
    double GetHeading();
    void SetPosition(double x, double y, double heading);
};