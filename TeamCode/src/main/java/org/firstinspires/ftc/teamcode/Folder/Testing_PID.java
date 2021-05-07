package org.firstinspires.ftc.teamcode.Folder;/*
package org.firstinspires.ftc.teamcode.DriveTrainAndPID.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DriveTrainAndPID.EncoderAndPIDDriveTrain;


//Back up Auton that goes to the wall side of the bridge, and parks there
@Config
@Autonomous (name = "TESTING PID")
@Disabled
public class Testing_PID extends LinearOpMode {
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    //initializaing the future variables
    private ElapsedTime runtime = new ElapsedTime();
    Motor LFMotor, LBMotor, RFMotor, RBMotor;
    RevIMU imu;
    EncoderAndPIDDriveTrain drive;
    private PIDController pidDrive;

    //no. of ticks per one revolution of the yellow jacket motors
    int Ticks_Per_Rev = 1316;

    @Override
    public void runOpMode(){
        // Initialize the hardware variables.
        LFMotor  = new Motor(hardwareMap,"LF Motor", Motor.GoBILDA.RPM_1150);
        LBMotor  = new Motor(hardwareMap,"LB Motor", Motor.GoBILDA.RPM_1150);
        RFMotor  = new Motor(hardwareMap,"RF Motor", Motor.GoBILDA.RPM_1150);
        RBMotor  = new Motor(hardwareMap,"RB Motor", Motor.GoBILDA.RPM_1150);
        imu = new RevIMU(hardwareMap, "imu");


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LFMotor.resetEncoder();
        LBMotor.resetEncoder();
        RFMotor.resetEncoder();
        RBMotor.resetEncoder();
        
        //Wheels on the robot funtions  
        drive = new EncoderAndPIDDriveTrain(LFMotor, LBMotor, RFMotor, RBMotor, imu);
        pidDrive = new PIDController(kP, kI, kD);

        pidDrive.setOutputRange(-1, 1);
        pidDrive.setInputRange(-100000, 100000);
        pidDrive.setTolerance(100);
        pidDrive.setSetpoint(500);
        pidDrive.reset();
        pidDrive.enable();


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Mode", "Init");
        telemetry.update();
        runtime.reset();
        waitForStart();

        //Running the code
        LFMotor.getCurrentPosition();
        while (opModeIsActive()) {
            do {
                int encode = (LFMotor.getCurrentPosition() + LBMotor.getCurrentPosition() + RFMotor.getCurrentPosition() + RBMotor.getCurrentPosition()) / 4;

                double power = pidDrive.performPID(encode);

                telemetry.addData("encode", encode);
                telemetry.addData("LF Motor",LFMotor.getCurrentPosition());
                telemetry.addData("LB Motor",LBMotor.getCurrentPosition());
                telemetry.addData("RF Motor",RFMotor.getCurrentPosition());
                telemetry.addData("RB Motor",RBMotor.getCurrentPosition());
                telemetry.addData("y", runtime);
                telemetry.addData("x",power);
                telemetry.log();
                telemetry.update();

                drive.DriveForward(power);
            } while (LFMotor.isBusy() && LBMotor.isBusy() && RFMotor.isBusy() && RBMotor.isBusy());
        }
    }

}*/