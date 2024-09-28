package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;
import static edu.wpi.first.units.Units.*;

public class TurnAuto extends Command {

    private final DriveSubsystem m_drive;
    private final Measure<Distance> m_arcDistence;
    private final double sign;
    private final double m_degrees;
    public TurnAuto(DriveSubsystem drive, double degrees) {
        m_drive = drive;
        m_degrees = degrees;
        m_arcDistence = Inches.of(DriveSubsystem.TRACK_WIDTH * Math.PI * Math.abs(degrees) / 360);
        sign = Math.signum(degrees); // (degrees >= 0) ? 1.0 : -1.0;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_drive.autoStop();
        m_drive.resetPose(new Pose2d());
        System.out.println("Turn Auto Initialized");
    }

    private boolean done;
    @Override
    public void execute() {
        var degrees = -m_drive.getPose().getRotation().getDegrees();
        // var distance = Math.abs(pose.getX());
        // var inches = Meters.of(distance).in(Inches);
        if (m_degrees < 0 && degrees < m_degrees) {
            done = true;
        }
        if (m_degrees > 0 && degrees > m_degrees) {
            done = true;
        }
        if (!done) {
            m_drive.autoDrive(sign * -0.20, sign * 0.20);
        } else {
            m_drive.autoStop();
            done = true;
            System.out.println("Turn Auto Stop");
        }
    }

    @Override
    public void end(boolean inturrupted) {
        if (inturrupted) {
            System.out.println("Turn Auto Interrupted");
        } else {
            System.out.println("Turn Auto Ended");
        }
        m_drive.autoStop();
    }

    @Override
    public boolean isFinished() {
        var pose = m_drive.getTranslation();
        var distance = Math.abs(pose.getX());
        var inches = Meters.of(distance).in(Inches);

        if (done) {
            System.out.println("Turn Auto Finished");
            return true;
        } else {
            return false;
        }
    }

}
