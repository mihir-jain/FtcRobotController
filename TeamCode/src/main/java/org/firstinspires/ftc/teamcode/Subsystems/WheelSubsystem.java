package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class WheelSubsystem extends SubsystemBase {
    private final Motor m_wheel;

    public WheelSubsystem(final HardwareMap hardwareMap, final String name, final boolean inverted) {
        m_wheel = new Motor(hardwareMap, name, Motor.GoBILDA.RPM_1150);
        m_wheel.setInverted(inverted);
        m_wheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_wheel.resetEncoder();
        m_wheel.setRunMode(Motor.RunMode.RawPower);
    }

    public void resetEncoder() {
        m_wheel.resetEncoder();
    }

    public Motor.Encoder getEncoderValue() {
        return m_wheel.encoder;
    }

    public void drivePower(double power, double damp) {
        power = damp * power;
        m_wheel.set(power);
    }
}
