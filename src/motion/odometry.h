#pragma once

enum class OdometryType {
    TRACKING_WHEELS = 0,
    X_DRIVE
};

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
    double m_wheelDiameter = 3.25;
    bool m_stopTask = false;
    OdometryType m_type;

    double m_lastForwardLeftDegrees = 0;
    double m_lastForwardRightDegrees = 0;
    double m_lastBackLeftDegrees = 0;
    double m_lastBackRightDegrees = 0;
public:
    Odometry(double verticalTrackingWheelOffset = 0, double horizontalTrackingWheelOffset = 0, OdometryType type = OdometryType::TRACKING_WHEELS, double wheelDiameter = 3.25) : m_verticalTrackingWheelOffset(verticalTrackingWheelOffset), m_horizontalTrackingWheelOffset(horizontalTrackingWheelOffset), m_type(type), m_wheelDiameter(wheelDiameter) {};
    void StartUpdating();
    void StopUpdating();
    double GetX();
    double GetY();
    double GetHeading();
    double GetWheelDiameter();
    void SetPosition(double x, double y, double heading);
};