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
    public TurnAuto(DriveSubsystem drive, int degrees) {
        m_drive = drive;
        m_arcDistence = Inches.of(DriveSubsystem.TRACK_WIDTH * Math.PI * Math.abs(degrees) / 360);
        sign = (degrees >= 0) ? 1.0 : -1.0;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_drive.autoStop();
        m_drive.resetPose(new Pose2d());
    }

    @Override
    public void execute() {
        var pose = m_drive.getTranslation();
        var distance = sign * pose.getX();
        var inches = Meters.of(distance).in(Inches);

        if (inches <= m_arcDistence.in(Inches)) {
            m_drive.autoDrive(sign * 0.25, sign * -0.25);
        } else {
            m_drive.autoStop();
        }
    }

    @Override
    public void end(boolean inturrupted) {
    }

    @Override
    public boolean isFinished() {
        var pose = m_drive.getTranslation();
        var distance = sign * pose.getX();
        var inches = Meters.of(distance).in(Inches);

        if (inches >= m_arcDistence.in(Inches)) {
            return true;
        } else {
            return false;
        }
    }

}
