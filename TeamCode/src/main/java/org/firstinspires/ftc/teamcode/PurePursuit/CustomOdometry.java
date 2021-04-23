package org.firstinspires.ftc.teamcode.PurePursuit;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CustomOdometry {

    private HolonomicOdometry m_robotOdometry;
    private OdometrySubsystem m_odometry;
    private MotorEx leftEncoder, rightEncoder, perpEncoder;

    public static double TRACKWIDTH = 15.75;
    public static double CENTER_WHEEL_OFFSET = 0.5;
    public static double WHEEL_DIAMETER = 1.49606;

    public static double negLeft = 1;
    public static double negRight = -1;
    public static double negRear = 1;

    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 360 * 4;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    public CustomOdometry(HardwareMap hardwareMap) {
        leftEncoder = new MotorEx(hardwareMap, "Elevator Motor");
        rightEncoder = new MotorEx(hardwareMap, "Conveyor Motor");
        perpEncoder = new MotorEx(hardwareMap, "Right Shooter Motor");

        leftEncoder.setDistancePerPulse(negLeft * DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(negRight * DISTANCE_PER_PULSE);
        perpEncoder.setDistancePerPulse(negRear * DISTANCE_PER_PULSE);

        m_robotOdometry = new HolonomicOdometry(
                leftEncoder::getDistance, rightEncoder::getDistance, perpEncoder::getDistance, TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        m_odometry = new OdometrySubsystem(m_robotOdometry);
    }

    public HolonomicOdometry getRobotOdometry() {
        return m_robotOdometry;
    }

    public double getX() {
        return getRobotOdometry().getPose().getX();
    }

    public double getY() {
        return getRobotOdometry().getPose().getY();
    }

    public double getAngle() {
        return getRobotOdometry().getPose().getRotation().getDegrees();
    }

    public OdometrySubsystem getOdometry() {
        return m_odometry;
    }
}
 