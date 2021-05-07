package org.firstinspires.ftc.teamcode.Folder.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Folder.Subsystems.DriveSubsystem;

public class TurningCommand extends CommandBase {
    private final DriveSubsystem m_drive;
    private final double m_angle;
    private final double m_power;
    private int[] direction = new int[4];
    private PIDController pidController;

    public TurningCommand(DriveSubsystem drive, double angle, double power) {
        m_drive = drive;
        m_angle = angle;
        m_power = power;
        if (m_angle > 180 || m_angle < 0) {
            direction[0] = 1;
            direction[1] = 1;
            direction[2] = -1;
            direction[3] = -1;
        } else {
            direction[0] = -1;
            direction[1] = -1;
            direction[2] = 1;
            direction[3] = 1;
        }

        pidController = new PIDController(.003, .00003, 0);
        pidController.reset();
        pidController.setTolerance(1);
        pidController.setSetPoint(angle);
        m_drive.resetAngle();
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_drive.resetAngle();
    }

    @Override
    public void execute() {
        double speed = pidController.calculate(m_drive.getAngle());
        if (speed > m_power) {
            speed = m_power;
        }

        m_drive.drive(speed, direction);
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetPoint() && m_drive.isDone();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, direction);
        pidController.reset();
    }
}
