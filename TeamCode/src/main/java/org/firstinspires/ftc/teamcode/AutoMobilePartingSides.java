package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.EncoderAndPIDDriveTrain;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

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

    private static double shooterSpeed = 0.63;
    double anglioso = 0;
    public static double ratioNumber = 1.2;
    private static double highGoalNumber = 0.6429;

    private WebcamName webcam1, webcam2;
    private SwitchableCamera switchableCamera;

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private PIDController pidRotate;

    List<VuforiaTrackable> allTrackables;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    VuforiaTrackables targetsUltimateGoal;

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
        imu = new RevIMU(hardwareMap, "imu 1");

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

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 2.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(webcam2, cameraLocationOnRobot);
        }

        targetsUltimateGoal.activate();

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

            switchableCamera.setActiveCamera(webcam2);

            telemetry.addData("Hoip: ", rec);
            telemetry.update();

            rightIntakeDownServo.setPosition(Servo.MAX_POSITION);
            leftIntakeDownServo.setPosition(Servo.MAX_POSITION);

            drive.DriveForwardDistance(2, 11);
            drive.StrafeLeftDistance(2, 15);
            drive.TurnLeftDistance(2, 2.2);
            //drive.DriveForwardDistance(2, 15);
            //Thread.sleep(500);

            /*targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            telemetry.addData("CAN I SEE THE TARGET? The answer has to be ", targetVisible);
            telemetry.update();*/

            drive.DriveBackwardDistance(2, 5);

            // Provide feedback as to where the robot is located (if we know).
            /*if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                double updatedX = (translation.get(0) / mmPerInch);

                double distance = Math.sqrt(Math.pow((translation.get(0) / mmPerInch), 2) + Math.pow((translation.get(1) / mmPerInch), 2));
                double distanceforspeed = Math.sqrt(Math.pow((translation.get(0)), 2) + Math.pow((translation.get(1)), 2)) / 1000;
                double highGoalVelocity = Math.sqrt((5.4228 * Math.pow(distanceforspeed, 2)) / (highGoalNumber - 0.3249 * distanceforspeed));

                double speedForHighGoal = Math.abs((highGoalVelocity * 2.23694 - 17.4) / 9.02) - 0.07;

                anglioso = rotation.thirdAngle - (updatedX / ratioNumber);
                if (anglioso < 0) {
                    drive.TurnLeftDegrees(0.75, -anglioso);
                } else if (anglioso > 0) {
                    drive.TurnRightDegrees(0.75, anglioso);
                }

                telemetry.addData("hoi, anglioso", anglioso);
                telemetry.update();

                //Thread.sleep(500);

                //shooterSpeed = speedForHighGoal;
            }*/

            rightShooterMotor.set(shooterSpeed);
            leftShooterMotor.set(shooterSpeed);
            Thread.sleep(1500);
            elevatorMotor.set(1.0);
            conveyorMotor.set(1.0);
            Thread.sleep(1000);
            Thread.sleep(3000);


            drive.DriveForwardDistance(1, 16);

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

                drive.StrafeRightDistance(2, 39);
            } else if (rec == "Single") {
                drive.DriveForwardDistance(2, 23);
                liftWobbleGoalServo.setPower(1.0);
                Thread.sleep(3000);
                wobbleGoalClawServo.setPosition(Servo.MIN_POSITION);
                liftWobbleGoalServo.setPower(0.0);
                drive.StrafeRightDistance(2, 13);
                drive.DriveBackwardDistance(2, 33);
            } else {
                drive.StrafeLeftDistance(2, 20);
                drive.TurnLeftDistance(2, 3);
                drive.DriveBackwardDistance(2,  3);
                liftWobbleGoalServo.setPower(1.0);
                Thread.sleep(3000);
                wobbleGoalClawServo.setPosition(Servo.MIN_POSITION);
                liftWobbleGoalServo.setPower(0.0);
            }
        }
        targetsUltimateGoal.deactivate();

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
        webcam1 = hardwareMap.get(WebcamName.class, "pog");
        webcam2 = hardwareMap.get(WebcamName.class, "gamer");
        parameters.cameraName = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(webcam1, webcam2);
        parameters.maxWebcamAspectRatio = 1920/1080;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        switchableCamera = (SwitchableCamera) vuforia.getCamera();
        switchableCamera.setActiveCamera(webcam1);

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