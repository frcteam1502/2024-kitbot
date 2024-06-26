package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

public class DriveSubsystem extends SubsystemBase {
   
   private final WPI_VictorSPX m_left;
   private final WPI_VictorSPX m_right;
   
    private final Pigeon2 m_gyro = new Pigeon2(0);

    private final DifferentialDriveOdometry m_odometry;
public DriveSubsystem(WPI_VictorSPX left, WPI_VictorSPX right){
    this.m_left = left;
    this.m_right = right;
    m_odometry= new DifferentialDriveOdometry(new Rotation2d(),0,0);


    
  AutoBuilder.configureRamsete(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getCurrentSpeeds, // Current ChassisSpeeds supplier
            this::drive, // Method that will drive the robot given ChassisSpeeds
            new ReplanningConfig(), // Default path replanning config. See the API for the options here
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }
public Pose2d getPose() {
    return m_odometry.getPoseMeters();
}
public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), null, pose);
}

public ChassisSpeeds getCurrentSpeeds() {

}

public void drive(ChassisSpeeds speeds) {

}

}