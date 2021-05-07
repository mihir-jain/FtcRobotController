package org.firstinspires.ftc.teamcode.Folder.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Folder.Subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem m_drive;
    private final double m_distance;
    private final double m_power;
    private int[] direction = new int[4];

    public DriveCommand(DriveSubsystem drive, double distance, double power) {
        m_drive = drive;
        m_distance = distance;
        m_power = power;
        direction[0] = 1;
        direction[1] = 1;
        direction[2] = 1;
        direction[3] = 1;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_drive.resetEncoders();
    }

    @Override
    public void execute() {
        m_drive.drive(m_power, m_distance, direction);
    }

    @Override
    public boolean isFinished() {
        return m_drive.isDone();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, direction);
    }
}
