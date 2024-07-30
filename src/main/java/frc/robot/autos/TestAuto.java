package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;
import static edu.wpi.first.units.Units.*;

public class TestAuto extends Command {
    private final DriveSubsystem m_drive;

    public TestAuto(DriveSubsystem drive) {
        m_drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_drive.tankDrive(0, 0);
        m_drive.resetPose(null);
    }

    @Override
    public void execute() {
        m_drive.tankDrive(0.1, 0.1);
    }

    @Override
    public void end(boolean inturrupted) {

    }

    @Override
    public boolean isFinished() {
        var pose = m_drive.getPose();
        var distance = pose.getX();
        var feet = Meters.of(distance).in(Feet);

        return feet >= 3;
    }

}