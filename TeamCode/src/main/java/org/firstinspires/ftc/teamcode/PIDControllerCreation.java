package org.firstinspires.ftc.teamcode;

// PID controller courtesy of Peter Tischler and WPI, with modifications.

public class PIDControllerCreation {

    private double m_positionError;
    private double m_velocityError;
    private final double m_period;
    private double m_maximumIntegral = 1.0;
    private double m_minimumIntegral = -1.0;
    private double m_maximumInput = 0.0;
    private double m_minimumInput = 0.0;
    private boolean m_continuous = false;
    private double m_P;
    private double m_I;
    private double m_D;
    private double m_positionTolerance = 0.05;
    private double m_velocityTolerance = Double.POSITIVE_INFINITY;
    private double m_setpoint = 0.0;
    private boolean m_enabled = false;
    private double m_prevError = 0.0;
    private double m_totalError = 0.0;
    private double m_maximumOutput = 1.0;
    private double m_minimumOutput = -1.0;
    private double m_measurement;


    public PIDControllerCreation(double kp, double ki, double kd) {
        this(kp, ki, kd, 0.02);
    }

    public PIDControllerCreation(double kp, double ki, double kd, double period) {
        m_P = kp;
        m_I = ki;
        m_D = kd;

        if (period <= 0) {
            throw new IllegalArgumentException("Controller period must be a non-zero positive number!");
        }
        m_period = period;
    }

    public void setPID(double kp, double ki, double kd) {
        m_P = kp;
        m_I = ki;
        m_D = kd;
    }

    public void setP(double kp) {
        m_P = kp;
    }

    public void setI(double ki) {
        m_I = ki;
    }

    public void setD(double kd) {
        m_D = kd;
    }

    public double getP() {
        return m_P;
    }

    public double getI() {
        return m_I;
    }

    public double getD() {
        return m_D;
    }

    public double getPeriod() {
        return m_period;
    }

    public void setSetpoint(double setpoint) {
        m_setpoint = setpoint;
    }

    public double getSetpoint() {
        return m_setpoint;
    }

    public void enableContinuousInput(double minimumInput, double maximumInput) {
        m_continuous = true;
        m_minimumInput = minimumInput;
        m_maximumInput = maximumInput;
    }

    public void disableContinuousInput() {
        m_continuous = false;
    }

    public boolean isContinuousInputEnabled() {
        return m_continuous;
    }

    public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
        m_maximumIntegral = maximumIntegral;
        m_minimumIntegral = minimumIntegral;
    }

    public void setTolerance(double positionTolerance) {
        this.setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    public void setTolerance(double positionTolerance, double velocityTolerance) {
        m_positionTolerance = positionTolerance;
        m_velocityTolerance = velocityTolerance;
    }

    public double getPositionError() {
        return m_positionError;
    }

    public double getVelocityError() {
        return m_velocityError;
    }

    public double calculate(double measurement, double setpoint) {
        this.setSetpoint(setpoint);
        return this.calculate(measurement);
    }

    public double calculate(double measurement) {
        if (m_enabled) {
            m_measurement = measurement;
            m_prevError = m_positionError;

            if (m_continuous) {
                m_positionError =
                        inputModulus(m_setpoint - measurement, m_minimumInput, m_maximumInput);
            } else {
                m_positionError = m_setpoint - measurement;
            }

            m_velocityError = (m_positionError - m_prevError) / m_period;

            if (m_I != 0) {
                m_totalError =
                        clamp(
                                m_totalError + m_positionError * m_period,
                                m_minimumIntegral / m_I,
                                m_maximumIntegral / m_I);
            }

            double output = m_P * m_positionError + m_I * m_totalError + m_D * m_velocityError;

            if (output > m_maximumOutput) {
                output = m_maximumOutput;
            } else if (output < m_minimumOutput) {
                output = m_minimumOutput;
            }

            return output;
        }
        else {
            return 0.0;
        }
    }

    public void reset() {
        this.disable();
        m_prevError = 0;
        m_totalError = 0;
    }

    public boolean atSetpoint() {
        double positionError;
        if (m_continuous) {
            positionError =
                    inputModulus(m_setpoint - m_measurement, m_minimumInput, m_maximumInput);
        } else {
            positionError = m_setpoint - m_measurement;
        }

        double velocityError = (positionError - m_prevError) / m_period;

        return Math.abs(positionError) < m_positionTolerance
                && Math.abs(velocityError) < m_velocityTolerance;

    }

    public void setOutputRange(double minimumOutput, double maximumOutput) {
        m_minimumOutput = minimumOutput;
        m_maximumOutput = maximumOutput;
    }

    public void enable() {
        m_enabled = true;
    }

    public void disable() {
        m_enabled = false;
    }

    public static double inputModulus(double input, double minimumInput, double maximumInput) {
        double modulus = maximumInput - minimumInput;

        // Wrap input if it's above the maximum input
        int numMax = (int) ((input - minimumInput) / modulus);
        input -= numMax * modulus;

        // Wrap input if it's below the minimum input
        int numMin = (int) ((input - maximumInput) / modulus);
        input -= numMin * modulus;

        return input;
    }

    public static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }
}