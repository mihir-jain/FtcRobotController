package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

public class Odometry extends OdometrySubsystem {
    /**
     * Make sure you are using the supplier version of the constructor
     *
     * @param odometry the odometry on the robot
     */
    public Odometry(HolonomicOdometry odometry) {
        super(odometry);
    }
}
