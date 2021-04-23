package org.firstinspires.ftc.teamcode.PurePursuit;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled
public class League2 extends CommandOpMode {
    private MecanumDrive m_robotDrive;
    private Motor bR, fL, fR, bL;

    @Override
    public void initialize() {
        bR = new Motor(hardwareMap, "RB Motor", Motor.GoBILDA.RPM_1150);
        fL = new Motor(hardwareMap, "LF Motor", Motor.GoBILDA.RPM_1150);
        fR = new Motor(hardwareMap, "RF Motor", Motor.GoBILDA.RPM_1150);
        bL = new Motor(hardwareMap, "LB Motor", Motor.GoBILDA.RPM_1150);

        // create our drive object
        m_robotDrive = new MecanumDrive(fL, fR, bL, bR);

        //reversing the motors that need to be reversed, otherwise it sets it as forward
        fL.setInverted(true);
        bL.setInverted(false);
        fR.setInverted(false);
        bR.setInverted(false);

        fL.encoder.reset();
        fR.encoder.reset();
        bL.encoder.reset();

        CustomOdometry x = new CustomOdometry(hardwareMap);
        OdometrySubsystem m_odometry = x.getOdometry();
        HolonomicOdometry m_robotOdometry = x.getRobotOdometry();

        //Waypoints
        Waypoint startBlueStart = new StartWaypoint(0, 0);

        Waypoint ringsPickStart = new StartWaypoint(10, 12);
        Waypoint ringsPickEnd = new EndWaypoint(0, 100, 0, 0.5, 0.5, 30, 0.8, 1);

        Waypoint checkRingsFarStart = new StartWaypoint(0, 12);
        Waypoint checkRingsFarEnd = new EndWaypoint(0, 12, 0, 0.5, 0.5, 30, 0.8, 1);

        Waypoint powerShots1Start = new StartWaypoint(33, -8);
        Waypoint powerShots1End = new EndWaypoint(33, -8, 0, 0.5, 0.5, 30, 0.8, 1);

        Waypoint powerShots2Start = new StartWaypoint(58, -8);
        Waypoint powerShots2End = new EndWaypoint(58, -8, 0, 0.5, 0.5, 30, 0.8, 1);

        Waypoint wobble21Start = new StartWaypoint(10, 32);
        Waypoint wobble21End = new EndWaypoint(10, 32, 0, 0.5, 0.5, 30, 0.8, 1);

        Waypoint wobble22Start = new StartWaypoint(10, 22);
        Waypoint wobble22End = new EndWaypoint(10, 31, 0, 0.5, 0.5, 30, 0.8, 1);

        Waypoint highGoalStart = new StartWaypoint(58, 12);
        Waypoint highGoalEnd = new EndWaypoint(58, 12, 0, 0.5, 0.5, 30, 0.8, 1);

        PurePursuitCommand ppCommand;

        //Actual path
        waitForStart();
        if(!isStopRequested()) {

            //Path 1
            ppCommand = new PurePursuitCommand(
                    m_robotDrive, m_odometry, startBlueStart, ringsPickEnd
            );

            schedule(ppCommand);

            //Detect rings
            int num = 4;
            Waypoint locationStart = new StartWaypoint(58, 27);
            Waypoint locationEnd = new EndWaypoint(58, 27, 0, 0.5, 0.5, 30, 0.8, 1);

            if (num == 1) {
                locationStart = new StartWaypoint(98, 35);
                locationEnd = new EndWaypoint(98, 35, 0, 0.5, 0.5, 30, 0.8, 1);
            } else if (num == 4) {
                locationStart = new StartWaypoint(98, 35);
                locationEnd = new EndWaypoint(98, 35, 0, 0.5, 0.5, 30, 0.8, 1);
            }


            /*//Path 2
            ppCommand = new PurePursuitCommand(
                    m_robotDrive, m_odometry, checkRingsFarStart, powerShots1End
            );
            schedule(ppCommand);

            //Path 3
            ppCommand = new PurePursuitCommand(
                    m_robotDrive, m_odometry, powerShots1Start, powerShots2End
            );
            schedule(ppCommand);

            //Path 4
            ppCommand = new PurePursuitCommand(
                    m_robotDrive, m_odometry, powerShots2Start, locationEnd
            );
            schedule(ppCommand);

            //Path 5
            ppCommand = new PurePursuitCommand(
                    m_robotDrive, m_odometry, locationStart, wobble21End
            );
            schedule(ppCommand);

            //Path 6
            ppCommand = new PurePursuitCommand(
                    m_robotDrive, m_odometry, wobble21Start, wobble22End
            );
            schedule(ppCommand);

            //Path 7
            ppCommand = new PurePursuitCommand(
                    m_robotDrive, m_odometry, wobble22Start, ringsPickEnd
            );
            schedule(ppCommand);

            //Path 8
            ppCommand = new PurePursuitCommand(
                    m_robotDrive, m_odometry, ringsPickStart, locationEnd
            );
            schedule(ppCommand);

            //Path 9
            ppCommand = new PurePursuitCommand(
                    m_robotDrive, m_odometry, locationStart, highGoalEnd
            );
            schedule(ppCommand);*/
        }
    }
}