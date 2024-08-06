package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;

public class TestAuto extends Command {
    private final DriveSubsystem m_drive;

    public TestAuto(DriveSubsystem drive) {
        m_drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_drive.tankDrive(0, 0);
        m_drive.resetPose(new Pose2d());
    }

    @Override
    public void execute() {
        m_drive.tankDrive(-0.25, -0.25);
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