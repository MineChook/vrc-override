
#pragma once

#include <cstdint>

template <typename ErrorType>
class ControllerData {
private:
    double m_kp;
    double m_ki;
    double m_kd;

    double m_distanceTolerance;
    double m_headingTolerance;

    ErrorType m_error;
    ErrorType m_lastError;
    ErrorType m_integral;
    ErrorType m_derivative;

public:
    ControllerData(double kp = 0, double ki = 0, double kd = 0, double distanceTolerance = 0, double headingTolerance = 0) {
        this->m_kp = kp;
        this->m_ki = ki;
        this->m_kd = kd;
        this->m_distanceTolerance = distanceTolerance;
        this->m_headingTolerance = headingTolerance;
    }

    double getKp() const {
        return m_kp;
    }

    double getKi() const {
        return m_ki;
    }

    double getKd() const {
        return m_kd;
    }

    double getDistanceTolerance() const {
        return m_distanceTolerance;
    }

    double getHeadingTolerance() const {
        return m_headingTolerance;
    }

    ErrorType getError() const {
        return m_error;
    }

    void setError(ErrorType error) {
        m_error = error;
    }

    ErrorType getLastError() const {
        return m_lastError;
    }

    void setLastError(ErrorType lastError) {
        m_lastError = lastError;
    }

    ErrorType getIntegral() const {
        return m_integral;
    }

    void setIntegral(ErrorType integral) {
        m_integral = integral;
    }

    ErrorType getDerivative() const {
        return m_derivative;
    }

    void setDerivative(ErrorType derivative) {
        m_derivative = derivative;
    }
};

class DriveControllerData {
private:
    uint8_t m_sensitivity;
    uint8_t m_maxSpeed;
    uint8_t m_deadzone;
    ControllerData<double> m_turningControllerData;

    double m_targetHeading = 0;

public:
    DriveControllerData(uint8_t sensitivity, uint8_t maxSpeed, uint8_t deadzone, ControllerData<double> turningControllerData) {
        this->m_sensitivity = sensitivity;
        this->m_maxSpeed = maxSpeed;
        this->m_deadzone = deadzone;
        this->m_turningControllerData = turningControllerData;
    }

    uint8_t getSensitivity() const {
        return m_sensitivity;
    }

    uint8_t getMaxSpeed() const {
        return m_maxSpeed;
    }

    uint8_t getDeadzone() const {
        return m_deadzone;
    }

    ControllerData<double> getTurningControllerData() const {
        return m_turningControllerData;
    }

    double getTargetHeading() const {
        return m_targetHeading;
    }

    void setTargetHeading(double targetHeading) {
        m_targetHeading = targetHeading;
    }
};