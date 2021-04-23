package org.firstinspires.ftc.teamcode.PurePursuit;

public class MathFunctions {
    //Angle from -180 - 180
    public static double AngleWrap(double angle) {
        while(angle < -Math.PI) {
            angle += 2 * Math.PI;
        }

        while(angle > Math.PI) {
            angle -= 2 * Math.PI;
        }

        return angle;
    }
}
