package org.firstinspires.ftc.teamcode.Folder;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.vision.UGRectDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EncoderAndPIDDriveTrain;

//Back up Auton that goes to the wall side of the bridge, and parks there

@Autonomous (name = "Back_Up_Back")
@Disabled
public class Auton extends LinearOpMode {

    //initializaing the future variables
    private ElapsedTime runtime = new ElapsedTime();
    Motor LFMotor, LBMotor, RFMotor, RBMotor, conveyorMotor, elevatorMotor, leftShooterMotor, rightShooterMotor;
    EncoderAndPIDDriveTrain drive;
    UGRectDetector UGRectDetector;
    RevIMU imu;

    //no. of ticks per one revolution of the yellow jacket motors
    int Ticks_Per_Rev = 1316;

    @Override
    public void runOpMode() throws InterruptedException{
        // Initialize the hardware variables.
        LFMotor  = new Motor(hardwareMap, "LF Motor", Motor.GoBILDA.RPM_1150);
        LBMotor  = new Motor(hardwareMap, "LB Motor", Motor.GoBILDA.RPM_1150);
        RFMotor  = new Motor(hardwareMap, "RF Motor", Motor.GoBILDA.RPM_1150);
        RBMotor  = new Motor(hardwareMap, "RB Motor", Motor.GoBILDA.RPM_1150);

        conveyorMotor = new Motor(hardwareMap, "Conveyor Motor", Motor.GoBILDA.RPM_435);
        elevatorMotor = new Motor(hardwareMap, "Elevator Motor", Motor.GoBILDA.RPM_84);
        leftShooterMotor = new Motor(hardwareMap, "Left Shooter Motor", Motor.GoBILDA.RPM_1150);
        rightShooterMotor = new Motor(hardwareMap, "Right Shooter Motor", Motor.GoBILDA.RPM_1150);
        imu = new RevIMU(hardwareMap, "imu");

        UGRectDetector = new UGRectDetector(hardwareMap, "pog");
        UGRectDetector.init();

        conveyorMotor.setInverted(true);
        elevatorMotor.setInverted(true);
        leftShooterMotor.setInverted(false);
        rightShooterMotor.setInverted(true);

        conveyorMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooterMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooterMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Wheels on the chassis functions
        drive = new EncoderAndPIDDriveTrain(LFMotor, LBMotor, RFMotor, RBMotor, imu);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Mode", "Init");
        telemetry.update();
        runtime.reset();
        waitForStart();

        //Running the code
        LFMotor.getCurrentPosition();
        if (opModeIsActive()) {
            //drive forward to get under the bridge

            drive.StrafeLeftDistance(2, 12);
            drive.DriveForwardDistance(1, 35);

            drive.StrafeRightDistance(1.5, 20);

            UGRectDetector.Stack stack = UGRectDetector.getStack();
            switch (stack) {
                case ZERO:
                    telemetry.addData("Status; ", "0");
                    break;
                case ONE:
                    telemetry.addData("Status; ", "1");
                    break;
                case FOUR:
                    telemetry.addData("Status; ", "4");
                    break;
                default:
                    telemetry.addData("Status; ", "defualt");
                    break;
            }

            /*drive.StrafeRightDistance(1.5, 35);
            drive.DriveBackwardDistance(1, 3);

            rightShooterMotor.set(0.8);
            leftShooterMotor.set(0.8);
            elevatorMotor.set(1.0);
            Thread.sleep(1000);
            conveyorMotor.set(1.0);
            Thread.sleep(500);
            conveyorMotor.set(0);
            Thread.sleep(1000);
            conveyorMotor.set(1.0);

            drive.DriveForwardDistance(1, 8);*/
        }
    }


}