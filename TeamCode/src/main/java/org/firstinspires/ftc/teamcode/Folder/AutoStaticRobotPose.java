package org.firstinspires.ftc.teamcode.Folder;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
@Disabled
public class AutoStaticRobotPose extends LinearOpMode {

    DcMotor LFMotor, LBMotor, RFMotor, RBMotor;
    private Motor leftEncoder, rightEncoder, perpEncoder;
    private HolonomicOdometry odometry;
    RevIMU imu;
    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry dashboard = dash.getTelemetry();
    public static double TRACKWIDTH = 15.75;
    public static double CENTER_WHEEL_OFFSET = 0.5;
    public static double WHEEL_DIAMETER = 1.49606;

    public static double negLeft = 1;
    public static double negRight = -1;
    public static double negRear = 1;

    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 360*4;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    @Override
    public void runOpMode() throws InterruptedException {
        leftEncoder = new Motor(hardwareMap, "Elevator Motor");
        rightEncoder = new Motor(hardwareMap, "Conveyor Motor");
        perpEncoder = new Motor(hardwareMap, "Right Shooter Motor");

        LFMotor  = hardwareMap.get(DcMotor.class, "LF Motor");
        LBMotor  = hardwareMap.get(DcMotor.class, "LB Motor");
        RFMotor  = hardwareMap.get(DcMotor.class, "RF Motor");
        RBMotor  = hardwareMap.get(DcMotor.class, "RB Motor");
        imu = new RevIMU(hardwareMap, "imu");

        leftEncoder.setDistancePerPulse(negLeft * DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(negRight * DISTANCE_PER_PULSE);
        perpEncoder.setDistancePerPulse(negRear * DISTANCE_PER_PULSE);

        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //setting the motor directions
        LFMotor.setDirection(DcMotor.Direction.REVERSE);
        LBMotor.setDirection(DcMotor.Direction.FORWARD);
        RFMotor.setDirection(DcMotor.Direction.REVERSE);
        RBMotor.setDirection(DcMotor.Direction.REVERSE);


        //defining and creating the IMU sensor for the robot to use
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu.init(parameters);

        odometry = new HolonomicOdometry(
                leftEncoder::getDistance,
                rightEncoder::getDistance,
                perpEncoder::getDistance,
                TRACKWIDTH,
                CENTER_WHEEL_OFFSET
        );

        waitForStart();
        PositionTracker.robotPose = odometry.getPose();
        dashboard.addData("X Position in inches: ", PositionTracker.robotPose.getX());
        dashboard.addData("Y Position in inches: ", PositionTracker.robotPose.getY());
        dashboard.addData("Rotational Position in degrees: ", PositionTracker.robotPose.getRotation().getDegrees());
        dashboard.update();

        int distance = 35;

        distance /= 2.6;

        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Diameter of wheel = 4in.  Circumference = 12.57; Ticks per revolution of goBilda motor = 1136
        //Ticks per inch = 1136/12.57 (approximately 90.37)
        int encoderDistance = (int) ((LFMotor.getCurrentPosition() + RBMotor.getCurrentPosition())/2 + distance * 90);

        //Set target position
        LFMotor.setTargetPosition(encoderDistance);
        LBMotor.setTargetPosition(encoderDistance);
        RFMotor.setTargetPosition(encoderDistance);
        RBMotor.setTargetPosition(encoderDistance);

        //set run to position mode
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        LFMotor.setPower(0.25);
        LBMotor.setPower(0.25);
        RFMotor.setPower(0.25);
        RBMotor.setPower(0.25);


        while (LFMotor.isBusy() && LBMotor.isBusy() && RFMotor.isBusy() && RBMotor.isBusy()) {
            odometry.updatePose();
            PositionTracker.robotPose = odometry.getPose();

            dashboard.addData("X Position in inches: ", PositionTracker.robotPose.getTranslation().getX());
            dashboard.addData("Y Position in inches: ", PositionTracker.robotPose.getTranslation().getY());
            dashboard.addData("Rotational Position in degrees: ", PositionTracker.robotPose.getRotation().getDegrees());
            dashboard.update();
            LFMotor.setPower(0.25);
            LBMotor.setPower(0.25);
            RFMotor.setPower(0.25);
            RBMotor.setPower(0.25);
        }

        //Stop and change modes back to normal
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RFMotor.setPower(0);
        RBMotor.setPower(0);

        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (!isStopRequested()) {
            // run autonomous

            // update positions
            odometry.updatePose();
            PositionTracker.robotPose = odometry.getPose();

            dashboard.addData("X Position in inches: ", PositionTracker.robotPose.getTranslation().getX());
            dashboard.addData("Y Position in inches: ", PositionTracker.robotPose.getTranslation().getY());
            dashboard.addData("Rotational Position in degrees: ", PositionTracker.robotPose.getRotation().getDegrees());
            dashboard.update();
        }

        if (isStopRequested()) {
            distance = -35;

            distance /= 2.6;

            LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //Diameter of wheel = 4in.  Circumference = 12.57; Ticks per revolution of goBilda motor = 1136
            //Ticks per inch = 1136/12.57 (approximately 90.37)
            encoderDistance = (int) ((LFMotor.getCurrentPosition() + RBMotor.getCurrentPosition()) / 2 + distance * 90);

            //Set target position
            LFMotor.setTargetPosition(encoderDistance);
            LBMotor.setTargetPosition(encoderDistance);
            RFMotor.setTargetPosition(encoderDistance);
            RBMotor.setTargetPosition(encoderDistance);

            //set run to position mode
            LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            LFMotor.setPower(-0.25);
            LBMotor.setPower(-0.25);
            RFMotor.setPower(-0.25);
            RBMotor.setPower(-0.25);


            while (LFMotor.isBusy() && LBMotor.isBusy() && RFMotor.isBusy() && RBMotor.isBusy()) {
                LFMotor.setPower(-0.25);
                LBMotor.setPower(-0.25);
                RFMotor.setPower(-0.25);
                RBMotor.setPower(-0.25);
            }

            //Stop and change modes back to normal
            LFMotor.setPower(0);
            LBMotor.setPower(0);
            RFMotor.setPower(0);
            RBMotor.setPower(0);

            LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

}
