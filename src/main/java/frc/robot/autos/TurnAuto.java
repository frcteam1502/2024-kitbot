package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;
import static edu.wpi.first.units.Units.*;

public class TurnAuto extends Command{
    
    private final DriveSubsystem m_drive;
    private double m_arcDistence;
    public TurnAuto(DriveSubsystem drive) {
        m_drive = drive;
        addRequirements(drive);

    }

    @Override
    public void initialize() {
        m_drive.tankDrive(0, 0);
        m_drive.resetPose(new Pose2d());
        m_arcDistence = DriveSubsystem.TRACK_WIDTH *Math.PI/4;
        
    }

    // @Override
    // public void execute() {
    // m_drive.tankDrive(-0.25, -0.25);

    // }

    @Override
    public void execute(){

        var pose = m_drive.getTranslation();
            var distance = pose.getX();
            var feet = Meters.of(distance).in(Feet);

            if (feet <= m_arcDistence) {
                m_drive.tankDrive(0.40, -0.40);
            } else {

                m_drive.tankDrive(0, 0);
            }
    }

    @Override
    public void end(boolean inturrupted) {

    }

    @Override
    public boolean isFinished() {
        var pose = m_drive.getPose();
        var distance = pose.getX();
        var feet = Meters.of(distance).in(Feet);

       if (feet >= m_arcDistence){
        return true;
    
       }
       else{
        return false;
       }
    }


}
    

