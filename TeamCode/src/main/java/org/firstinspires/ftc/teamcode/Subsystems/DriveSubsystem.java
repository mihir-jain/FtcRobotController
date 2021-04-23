package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.RevIMU;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class DriveSubsystem extends SubsystemBase {
    private WheelSubsystem LFMotor, LRMotor, RFMotor, RRMotor;
    private PIDController pidController;
    private RevIMU imu;
    private double power;
    private int[] direction = new int[4];
    double damp = 0.5;

    public DriveSubsystem() {
        LFMotor = new WheelSubsystem(hardwareMap, "LF Motor", false);
        LRMotor = new WheelSubsystem(hardwareMap, "LB Motor", false);
        RFMotor = new WheelSubsystem(hardwareMap, "RF Motor", true);
        RRMotor = new WheelSubsystem(hardwareMap, "RB Motor", true);
        imu = new RevIMU(hardwareMap, "imu");
        imu.init();

        pidController = new PIDController(.003, .00003, 0);
        pidController.reset();
        pidController.setTolerance(1);
        resetEncoders();

        power = 0;
        direction[0] = 1;
        direction[1] = 1;
        direction[2] = 1;
        direction[3] = 1;
    }

    /**
     * @param direction takes in an integer array with the direction that each wheel should be turning in the order,
     *                  the Left Front Motor, the Left Rear Motor, the Right Front Motor, and finally the Right Rear Motor
     */
    public void drive(double power, int[] direction) {
        LFMotor.drivePower(direction[0] * power, damp);
        LRMotor.drivePower(direction[1] * power, damp);
        RFMotor.drivePower(direction[2] * power, damp);
        RRMotor.drivePower(direction[3] * power, damp);
    }

    /**
     * @param direction takes in an integer array with the direction that each wheel should be turning in the order,
     *                  the Left Front Motor, the Left Rear Motor, the Right Front Motor, and finally the Right Rear Motor
     */
    public void drive(double power, double distance, int[] direction) {
        pidController.setSetPoint(distance);
        this.power = power;
        this.direction = direction;
    }

    public void resetEncoders() {
        LFMotor.resetEncoder();
        LRMotor.resetEncoder();
        RFMotor.resetEncoder();
        RRMotor.resetEncoder();
    }

    public boolean isDone() {
        return pidController.atSetPoint();
    }

    public double getAngle() {
        return imu.getAngles()[0];
    }

    public void resetAngle() {
        imu.reset();
    }

    public double getEncoderValues() {
        double sum = 0;
        sum += LFMotor.getEncoderValue().getPosition();
        sum += LRMotor.getEncoderValue().getPosition();
        sum += RFMotor.getEncoderValue().getPosition();
        sum += RRMotor.getEncoderValue().getPosition();
        return sum;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double pow = pidController.calculate(getEncoderValues());
        if (pow > power && power > 0) {
            pow = power;
        } else if (pow < power && power < 0) {
            pow = power;
        }

        drive(pow * damp, direction);
    }
}
