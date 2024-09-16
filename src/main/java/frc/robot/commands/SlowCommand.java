package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;

public class SlowCommand  extends Command {
    private final DriveSubsystem m_driveSubsystem;

    public SlowCommand(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_driveSubsystem.setMaxOutput(0.25);
    }

    @Override
    public void end(boolean inturrupted) {
        m_driveSubsystem.setMaxOutput(1.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
