package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.EncoderAndPIDDriveTrain;

import java.util.List;

//Back up Auton that goes to the wall side of the bridge, and parks there

@Autonomous(name="League 2 Auton")
//@Disabled
public class AutoMobilePartingSides extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AdK8BJf/////AAABmYCQFYMhCUEGpGiqBsjt6S9yKYcJbGmiZ8d9viFyvxFFTKpiCBwppicoI9FIGnm94cMjowewKG6d+1qKG55H92H6z2NVPrO4tplSO73k3cADtvGj/Zf9ennYyphiQdOJQSty+0MhKTcPUL9BokHQauvZR5v/mmYt+wGaoGuKB6jwprg7XGCR11UvFtafrntEn2p6EMMGy0ctEpA8dMIV0qT4pGi5w6/xve/yBegOt/mBbkaFViA8he6YjJHfS3xAGUShtWhgcPmqeM2c4nkDFfRxRhtBWPIgdc2Wu2Ud/kcw3SHId0DGSOauW6YWVnYGv7FJ5EzCDXYfmttCQEw5P9Rku0RL5um/e6yDNvWbRlmD";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    //initializaing the future variables
    private ElapsedTime runtime = new ElapsedTime();
    Motor LFMotor, LBMotor, RFMotor, RBMotor, conveyorMotor, elevatorMotor, leftShooterMotor, rightShooterMotor;
    private Servo rightIntakeDownServo, leftIntakeDownServo, wobbleGoalClawServo;
    private CRServo liftWobbleGoalServo;
    EncoderAndPIDDriveTrain drive;
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

        liftWobbleGoalServo = hardwareMap.crservo.get("Lift Wobble Goal Servo");
        wobbleGoalClawServo = hardwareMap.get(Servo.class, "Wobble Goal Claw Servo");
        rightIntakeDownServo = hardwareMap.get(Servo.class, "Right Intake Down Servo");
        leftIntakeDownServo = hardwareMap.get(Servo.class, "Left Intake Down Servo");
        imu = new RevIMU(hardwareMap, "imu");

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

        rightIntakeDownServo.setDirection(Servo.Direction.REVERSE);
        leftIntakeDownServo.setDirection(Servo.Direction.FORWARD);
        wobbleGoalClawServo.setDirection(Servo.Direction.FORWARD);
        liftWobbleGoalServo.setDirection(CRServo.Direction.FORWARD);

        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Mode", "Init");
        telemetry.update();
        runtime.reset();
        waitForStart();

        //Running the code
        if (opModeIsActive()) {
            //drive forward to get under the bridge
            //forward, lefts, forward, lefts, shoot

            //drive.StrafeLeftDistance(2, 10);
            //drive.TurnLeftDistance(2, 15);

            drive.DriveForwardDistance(1, 27);
            String rec = "";
            for (int j = 0; j < 10000001; j++) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            rec = recognition.getLabel();
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                        }
                        telemetry.update();
                    }
                }

                if (rec == "Quad" || rec == "Single") {
                    break;
                }
            }

            telemetry.addData("Hoip: ", rec);
            telemetry.update();

            drive.DriveForwardDistance(2, 11);
            drive.StrafeLeftDistance(2, 15);

            drive.TurnLeftDistance(2, 2);
            rightShooterMotor.set(0.631);
            leftShooterMotor.set(0.631);
            Thread.sleep(1000);
            elevatorMotor.set(1.0);
            conveyorMotor.set(1.0);
            Thread.sleep(1000);
            Thread.sleep(3000);


            drive.DriveForwardDistance(1, 11);

            rightShooterMotor.set(0);
            leftShooterMotor.set(0);
            elevatorMotor.set(0);
            conveyorMotor.set( 0);
            if (rec == "Quad") {
                drive.DriveForwardDistance(2, 28);
                drive.TurnRightDistance(2, 15);
                drive.DriveBackwardDistance(2, 4);
                liftWobbleGoalServo.setPower(1.0);
                Thread.sleep(3000);
                wobbleGoalClawServo.setPosition(Servo.MIN_POSITION);
                liftWobbleGoalServo.setPower(0.0);

                drive.StrafeRightDistance(2, 36);
            } else if (rec == "Single") {
                drive.DriveForwardDistance(2, 23);
                liftWobbleGoalServo.setPower(1.0);
                Thread.sleep(3000);
                wobbleGoalClawServo.setPosition(Servo.MIN_POSITION);
                liftWobbleGoalServo.setPower(0.0);
                drive.StrafeRightDistance(2, 13);
                drive.DriveBackwardDistance(2, 28);
            } else {
                drive.StrafeLeftDistance(2, 20);
                drive.TurnLeftDistance(2, 1);
                drive.DriveBackwardDistance(2,  3);
                liftWobbleGoalServo.setPower(1.0);
                Thread.sleep(3000);
                wobbleGoalClawServo.setPosition(Servo.MIN_POSITION);
                liftWobbleGoalServo.setPower(0.0);
            }
        }

        if (tfod != null) {
        tfod.shutdown();
        }

    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "pog");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}