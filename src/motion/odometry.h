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

    /**
     * @brief Construct a new Odometry object
     * 
     * @param verticalLeftTrackingWheelOffset 
     * @param verticalRightTrackingWheelOffset 
     * @param horizontalTrackingWheelOffset 
     * @param verticalWheelDiameter 
     * @param horizontalWheelDiameter 
     */
    Odometry(double verticalLeftTrackingWheelOffset = 0, double verticalRightTrackingWheelOffset = 0, double horizontalTrackingWheelOffset = 0, double verticalWheelDiameter = 3.25, double horizontalWheelDiameter = 3.25) : m_verticalLeftTrackingWheelOffset(verticalLeftTrackingWheelOffset), m_verticalRightTrackingWheelOffset(verticalRightTrackingWheelOffset), m_horizontalTrackingWheelOffset(horizontalTrackingWheelOffset), m_verticalWheelDiameter(verticalWheelDiameter), m_horizontalWheelDiameter(horizontalWheelDiameter) {};
    
    /**
     * @brief Starts the Odometry Update that keeps track of the robot's position
     * 
     */
    void StartUpdating();

    /**
     * @brief Stops the Odometry Update
     * 
     */
    void StopUpdating();

    /**
     * @brief Get the X Position
     * 
     * @return Return the X Position of the Robot in Inches
     */
    double GetX();

    /**
     * @brief Get the Y Position
     * 
     * @return Return the Y Position of the Robot in Inches
     */
    double GetY();

    /**
     * @brief Get the Heading Position
     * 
     * @return Return the Heading in Radians
     */
    double GetHeading();

    /**
     * @brief Converts Rotation of the Sensor to Distance the Robot has traveled
     * 
     * @param degrees 
     * @param wheelType 
     * @return double 
     */
    double DegreesToDistance(double degrees, uint8_t wheelType);

    /**
     * @brief Set the Robots Position from X, Y in inches and heading as Radians
     * 
     * @param x 
     * @param y 
     * @param heading 
     */
    void SetPosition(double x, double y, double heading);
};