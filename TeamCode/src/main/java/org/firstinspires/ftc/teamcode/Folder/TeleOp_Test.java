package org.firstinspires.ftc.teamcode.Folder;

//importing the statements for the code below

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//importing the statements for the code below


@TeleOp(name="TeleOp Encoders", group="Iterative TeamCode")
@Disabled
public class TeleOp_Test extends OpMode {

    //defining all of the variables needed for the code
    private ElapsedTime runtime = new ElapsedTime();
    private Motor LFMotor, LBMotor, RFMotor, RBMotor, conveyorMotor, elevatorMotor, rightShooterMotor;
    private double speed = 0.5;

    // define our trackwidth
    static final double TRACKWIDTH = 14.5;

    // convert ticks to inches
    static final double TICKS_TO_INCHES = 76.6;

    static final double centerWheelOffset = 0.5;

    private HolonomicOdometry diffOdom;


    @Override
    public void init() throws IllegalArgumentException {

        //grabbing the hardware from the expansion hubs, and the configuration
        LFMotor  = new Motor(hardwareMap, "LF Motor", Motor.GoBILDA.RPM_1150);
        LBMotor  = new Motor(hardwareMap, "LB Motor", Motor.GoBILDA.RPM_1150);
        RFMotor  = new Motor(hardwareMap, "RF Motor", Motor.GoBILDA.RPM_1150);
        RBMotor  = new Motor(hardwareMap, "RB Motor", Motor.GoBILDA.RPM_1150);
        conveyorMotor = new Motor(hardwareMap, "Conveyor Motor", Motor.GoBILDA.RPM_435);
        elevatorMotor = new Motor(hardwareMap, "Elevator Motor", Motor.GoBILDA.RPM_84);
        rightShooterMotor = new Motor(hardwareMap, "Right Shooter Motor", Motor.GoBILDA.RPM_1150);


        //reversing the motors that need to be reversed, otherwise it sets it as forward
        LFMotor.setInverted(false);
        LBMotor.setInverted(false);
        RFMotor.setInverted(true);
        RBMotor.setInverted(true);

        conveyorMotor.resetEncoder();
        elevatorMotor.resetEncoder();
        rightShooterMotor.resetEncoder();

        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {}

    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {

        //defining the value to get from phones
        double LFPower, LBPower, RFPower, RBPower, xValue, turnValue, yValue;


        //getting the movement values from the gamepad
        yValue = gamepad1.left_stick_y;
        turnValue = gamepad1.right_stick_x;
        xValue = gamepad1.left_stick_x;

        //getting the values for the powers for each motor
        LFPower = Range.clip(-yValue + turnValue + xValue,-1,1);
        LBPower = Range.clip(-yValue + turnValue - xValue,-1,1);
        RBPower = Range.clip(-yValue - turnValue + xValue,-1,1);
        RFPower = Range.clip(-yValue - turnValue - xValue,-1,1);

        //applying the ramping up and ramping down features
        if (LFPower < 0){
            LFPower = (float) Math.pow(Math.abs(LFPower),2);
        } else if (LFPower > 0){
            LFPower = (float) -Math.pow(Math.abs(LFPower),2);
        }

        if (LBPower < 0){
            LBPower = (float) -Math.pow(Math.abs(LBPower),2);
        } else if (LBPower > 0){
            LBPower = (float) Math.pow(Math.abs(LBPower),2);
        }

        if (RFPower < 0){
            RFPower = (float) -Math.pow(Math.abs(RFPower),2);
        } else if (RFPower > 0){
            RFPower = (float) Math.pow(Math.abs(RFPower),2);
        }

        if (RBPower < 0){
            RBPower = (float) -Math.pow(Math.abs(RBPower),2);
        } else if (RBPower > 0){
            RBPower = (float) Math.pow(Math.abs(RBPower),2);
        }

        //game pad 1 left trigger is intake and right trigger is shooting
        //trigger values are all in the float
        //need to set up motors, set their inversion factor, set speeds, based on trigger values
        //convereyr is counter, elevator is counter, shooting right is counter and shooting left is clock
        //


        if (gamepad1.a){
            speed = 0.3;
        } else{
            speed = 0.5;
        }


        //setting the powers for each of the motors
        LBMotor.setRunMode(Motor.RunMode.RawPower);
        LFMotor.setRunMode(Motor.RunMode.RawPower);
        RFMotor.setRunMode(Motor.RunMode.RawPower);
        RBMotor.setRunMode(Motor.RunMode.RawPower);

        LFMotor.set(Range.clip(LFPower, -speed, speed));
        LBMotor.set(Range.clip(LBPower, -speed, speed));
        RFMotor.set(Range.clip(RFPower, -speed, speed));
        RBMotor.set(Range.clip(RBPower, -speed, speed));

        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addData("Left: ", conveyorMotor.encoder.getDistance());
        telemetry.addData("Left: ", conveyorMotor.encoder.getRevolutions());
        telemetry.addData("Left: ", conveyorMotor.encoder.getPosition());
        telemetry.addData("Right: ", elevatorMotor.encoder.getDistance());
        telemetry.addData("Right: ", elevatorMotor.encoder.getRevolutions());
        telemetry.addData("Right: ", elevatorMotor.encoder.getPosition());
        telemetry.addData("Rear: ", rightShooterMotor.encoder.getDistance());
        telemetry.addData("Rear: ", rightShooterMotor.encoder.getRevolutions());
        telemetry.addData("Rear: ", rightShooterMotor.encoder.getPosition());
    }


    @Override
    public void stop() {
    }
}