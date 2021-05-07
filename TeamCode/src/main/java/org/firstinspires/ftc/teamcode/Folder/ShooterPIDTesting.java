package org.firstinspires.ftc.teamcode.Folder;

/*import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;*/
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


//Back up Auton that goes to the wall side of the bridge, and parks there
//@Config
@Autonomous (name = "TESTING PID")
@Disabled
public class ShooterPIDTesting extends LinearOpMode {
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    //initializaing the future variables
    private ElapsedTime runtime = new ElapsedTime();
    MotorEx leftShooterMotor, rightShooterMotor;

    @Override
    public void runOpMode(){
        // Initialize the hardware variables.
        leftShooterMotor = new MotorEx(hardwareMap, "Elevator Motor", Motor.GoBILDA.RPM_84);
        rightShooterMotor = new MotorEx(hardwareMap, "Right Shooter Motor", Motor.GoBILDA.RPM_1150);

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftShooterMotor.setInverted(false);
        rightShooterMotor.setInverted(true);

        leftShooterMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooterMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //leftShooterMotor.setRunMode(Motor.RunMode.RawPower);
        //rightShooterMotor.setRunMode(Motor.RunMode.RawPower);

        //leftShooterMotor.setVeloCoefficients(kP, kD, kI);
        //rightShooterMotor.setVeloCoefficients(kP, kD, kI);
        waitForStart();

        //Running the code
        while (opModeIsActive()) {
            leftShooterMotor.set(0.8);
            rightShooterMotor.set(0.8);
            telemetry.addData("Left Shooter Motor Error", leftShooterMotor.getVelocity() - 0.8 * leftShooterMotor.ACHIEVABLE_MAX_TICKS_PER_SECOND);
            telemetry.addData("Right Shooter Motor Error", rightShooterMotor.getVelocity() - 0.8 * rightShooterMotor.ACHIEVABLE_MAX_TICKS_PER_SECOND);
            telemetry.addData("Left Shooter Motor", leftShooterMotor.getVelocity());
            telemetry.addData("Right Shooter Motor", rightShooterMotor.getVelocity());
            telemetry.log();
            telemetry.update();
        }
    }

}