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
        m_drive.autoStop();
        m_drive.resetPose(new Pose2d());
        System.out.println("Forward Auto Initialized");
    }

    @Override
    public void execute(){
        var pose = m_drive.getPose();
        var distance = pose.getX();
        var feet = Meters.of(distance).in(Feet);

        if (feet <= 3) {
            m_drive.autoDrive(0.25, 0.25);
        } else {
            m_drive.autoStop();
        }
    }
    
    @Override
    public void end(boolean inturrupted) {
        m_drive.autoStop();
    }

    @Override
    public boolean isFinished() {
        var pose = m_drive.getPose();
        var distance = pose.getX();
        var feet = Meters.of(distance).in(Feet);
        if (feet >= 3) {
            System.out.println("Forward Auto Finished");
            return true;
        }
        return false;
    }

}