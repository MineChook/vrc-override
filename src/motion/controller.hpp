
#pragma once

#include <cstdint>

class ControllerData {
private:
    double m_kp;
    double m_ki;
    double m_kd;

    double m_error;
    double m_lastError;
    double m_integral;
    double m_derivative;

    double m_positionX;
    double m_positionY;

public:
    ControllerData(double kp = 0, double ki = 0, double kd = 0) {
        this->m_kp = kp;
        this->m_ki = ki;
        this->m_kd = kd;
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

    double getError() const {
        return m_error;
    }

    void setError(double error) {
        m_error = error;
    }

    double getLastError() const {
        return m_lastError;
    }

    void setLastError(double lastError) {
        m_lastError = lastError;
    }

    double getIntegral() const {
        return m_integral;
    }

    void setIntegral(double integral) {
        m_integral = integral;
    }

    double getDerivative() const {
        return m_derivative;
    }

    void setDerivative(double derivative) {
        m_derivative = derivative;
    }
};

class DriveControllerData {
private:
    uint8_t m_sensitivity;
    uint8_t m_maxSpeed;
    uint8_t m_deadzone;
    ControllerData m_turningControllerData;

    double m_targetHeading = 0;

public:
    DriveControllerData(uint8_t sensitivity, uint8_t maxSpeed, uint8_t deadzone, ControllerData turningControllerData) {
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

    ControllerData getTurningControllerData() const {
        return m_turningControllerData;
    }

    double getTargetHeading() const {
        return m_targetHeading;
    }

    void setTargetHeading(double targetHeading) {
        m_targetHeading = targetHeading;
    }
};