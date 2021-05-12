package org.firstinspires.ftc.teamcode;

// PID controller courtesy of Peter Tischler and WPI, with modifications.

public class PIDControllerCreation {
    private double m_positionError; //The error in the position of the bot, either transitionally and rotationally.
    private double m_velocityError; //The error in the velocity of the motor.
    private final double m_period; //The amount of seconds between when the PID loop is called.
    private double m_maximumIntegral = 1.0; //The maximum value of the integral, to be multiplied by the kI value.
    private double m_minimumIntegral = -1.0; //The minimum value of the integral, to be multiplied by the kI value.
    private double m_maximumInput = 0.0; //The maximum input value when continuous mode in enabled as a method to find the smallest angle needed to move.
    private double m_minimumInput = 0.0; //The minimum input value when continuous mode in enabled as a method to find the smallest angle needed to move.
    private boolean m_continuous = false; //When the continuous value is enabled or disabled, to wrap it and find the true smallest distance to move.
    private double m_P; //The p coefficient to be multiplied by the error.
    private double m_I; //The i coefficient to be multiplied by integral of the error or the total area under the curve.
    private double m_D; //The d coefficient to be multiplied by the derivative of the error or the current slope of the tangent line of the graph.
    private double m_positionTolerance = 0.05; //The overshoot that is allowed in the position to be counted as there.
    private double m_velocityTolerance = Double.POSITIVE_INFINITY; //The overshoot in the velocity that is allowed for the velocity to be counted as there.
    private double m_setpoint = 0.0; //Where you need to go in the position, so either the rotation or the translation.
    private boolean m_enabled = false; //Whether or not the PID controller is enabled. If it isn't, then it is going to return a speed of 0.
    private double m_prevError = 0.0; //The previous error, or the error that was just there.
    private double m_totalError = 0.0; //The total error so far, or the area under the curve. This is the integral of the error.
    private double m_maximumOutput = 1.0; //The maximum output, to be capped if the speed is over this speed.
    private double m_minimumOutput = -1.0; //The minimum output, to be capped if the speed is under this speed.
    private double m_measurement; //The current position

    /**
     * This initializes the PID, you need to tune and obtain the kp, ki and kd coefficients.
     *
     * @param kp This is the proportional coefficient, to be multiplied by the error.
     * @param ki This is the integral coefficient to be multiplied by integral of the error or the total area under the curve/total error.
     * @param kd This is the derivative coefficient to be multiplied by the derivative of the error or the current slope of the tangent line of the graph.
     */
    public PIDControllerCreation(double kp, double ki, double kd) {
        this(kp, ki, kd, 0.02);
    }

    /**
     * This initializes the PID, you need to tune and obtain the kp, ki and kd coefficients.
     *
     * @param kp This is the proportional coefficient, to be multiplied by the error.
     * @param ki This is the integral coefficient to be multiplied by integral of the error or the total area under the curve/total error.
     * @param kd This is the derivative coefficient to be multiplied by the derivative of the error or the current slope of the tangent line of the graph.
     * @param period This is the amount of seconds between when the PID loop is called.
     */
    public PIDControllerCreation(double kp, double ki, double kd, double period) {
        m_P = kp;
        m_I = ki;
        m_D = kd;

        if (period <= 0) {
            throw new IllegalArgumentException("Controller period must be a non-zero positive number!");
        }
        m_period = period;
    }

    /**
     * This changes the PID coefficients, you need to tune and obtain the kp, ki and kd coefficients.
     *
     * @param kp This is the proportional coefficient, to be multiplied by the error.
     * @param ki This is the integral coefficient to be multiplied by integral of the error or the total area under the curve/total error.
     * @param kd This is the derivative coefficient to be multiplied by the derivative of the error or the current slope of the tangent line of the graph.
     */
    public void setPID(double kp, double ki, double kd) {
        m_P = kp;
        m_I = ki;
        m_D = kd;
    }

    /**
     * This changes the P coefficient, you need to tune and obtain the kp coefficient.
     *
     * @param kp This is the proportional coefficient, to be multiplied by the error.
     */
    public void setP(double kp) {
        m_P = kp;
    }

    /**
     * This changes the I coefficient, you need to tune and obtain the kd coefficient.
     *
     * @param ki This is the integral coefficient to be multiplied by integral of the error or the total area under the curve/total error.
     */
    public void setI(double ki) {
        m_I = ki;
    }

    /**
     * This changes the D coefficient, you need to tune and obtain the kd coefficient.
     *
     * @param kd This is the derivative coefficient to be multiplied by the derivative of the error or the current slope of the tangent line of the graph.
     */
    public void setD(double kd) {
        m_D = kd;
    }

    /**
     * This returns the current P coefficient value.
     *
     * @return This returns the proportional coefficient, to be multiplied by the error as a double.
     */
    public double getP() {
        return m_P;
    }

    /**
     * This returns the current I coefficient value.
     *
     * @return This returns the integral coefficient to be multiplied by integral of the error or the total area under the curve/total error as a double.
     */
    public double getI() {
        return m_I;
    }

    /**
     * This returns the current D coefficient value.
     *
     * @return This returns the derivative coefficient to be multiplied by the derivative of the error or the current slope of the tangent line of the graph as a double.
     */
    public double getD() {
        return m_D;
    }

    /**
     * This returns the current period of the program.
     *
     * @return This returns the period, or the amount of seconds between when the PID loop is called as a double.
     */
    public double getPeriod() {
        return m_period;
    }

    /**
     * This sets the current setpoint for the PID Controller to go to.
     *
     * @param setpoint This is the setpoint, or the target velocity / position / angle.
     */
    public void setSetpoint(double setpoint) {
        m_setpoint = setpoint;
    }

    /**
     * This gets the current setpoint that the PID Controller is going to.
     *
     * @return This returns the setpoint, or the target velocity / position / angle as a double.
     */
    public double getSetpoint() {
        return m_setpoint;
    }

    /**
     * This allows the ability to have a continuous input, using the minimum and maximum inputs. This allows it to find the smallest distance between two points.
     *
     * @param minimumInput This is the minimum input, or the smallest value that an input can have
     * @param maximumInput This is the maximum input or the largest value that an input can have
     */
    public void enableContinuousInput(double minimumInput, double maximumInput) {
        m_continuous = true;
        m_minimumInput = minimumInput;
        m_maximumInput = maximumInput;
    }

    /**
     * This disables the continuous input, making it so that it will get the distance between a max and min
     */
    public void disableContinuousInput() {
        m_continuous = false;
    }

    /**
     * This will return if the continuous input has been enabled
     *
     * @return It will return a boolean stating if the continuous input has been enabled
     */
    public boolean isContinuousInputEnabled() {
        return m_continuous;
    }

    /**
     *
     *
     * @param minimumIntegral
     * @param maximumIntegral
     */
    public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
        m_maximumIntegral = maximumIntegral;
        m_minimumIntegral = minimumIntegral;
    }

    /**
     *
     *
     * @param positionTolerance
     */
    public void setTolerance(double positionTolerance) {
        this.setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    /**
     *
     *
     * @param positionTolerance
     * @param velocityTolerance
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        m_positionTolerance = positionTolerance;
        m_velocityTolerance = velocityTolerance;
    }

    /**
     *
     *
     * @return
     */
    public double getPositionError() {
        return m_positionError;
    }

    /**
     *
     *
     * @return
     */
    public double getVelocityError() {
        return m_velocityError;
    }

    /**
     *
     *
     * @param measurement
     * @param setpoint
     * @return
     */
    public double calculate(double measurement, double setpoint) {
        this.setSetpoint(setpoint);
        return this.calculate(measurement);
    }

    /**
     *
     *
     * @param measurement
     * @return
     */
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

    /**
     *
     */
    public void reset() {
        this.disable();
        m_prevError = 0;
        m_totalError = 0;
    }

    /**
     *
     *
     * @return
     */
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

    /**
     *
     *
     * @param minimumOutput
     * @param maximumOutput
     */
    public void setOutputRange(double minimumOutput, double maximumOutput) {
        m_minimumOutput = minimumOutput;
        m_maximumOutput = maximumOutput;
    }

    /**
     *
     */
    public void enable() {
        m_enabled = true;
    }

    /**
     *
     */
    public void disable() {
        m_enabled = false;
    }

    /**
     *
     *
     * @param input
     * @param minimumInput
     * @param maximumInput
     * @return
     */
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

    /**
     *
     *
     * @param value
     * @param low
     * @param high
     * @return
     */
    public static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }
}