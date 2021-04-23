package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.AngleWrap;

public class RobotMovement {
    public static double[] goToPosition(double x, double y, double movementSpeed, HardwareMap hardwareMap) {
        CustomOdometry co = new CustomOdometry(hardwareMap);
        double robotX = co.getX();
        double robotY = co.getY();
        double robotAngle = co.getAngle();

        double absoluteAngleToTarget = Math.atan2(y - robotX, x - robotY);
        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (robotAngle - Math.toRadians(90)));
        double distanceToTarget = Math.hypot(x - robotX, y - robotY);

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        return new double[]{movementXPower, movementYPower};
    }
}