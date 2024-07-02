package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import com.revrobotics.RelativeEncoder;

public class DriveSubsystem extends SubsystemBase {
   
    private final WPI_VictorSPX m_left;
    private final WPI_VictorSPX m_right;

    private Pose2d m_pose;

    private final Pigeon2 m_gyro = new Pigeon2(0);
    private final Encoder m_leftEncoder;
    private final Encoder m_rightEncoder;

    
    private final DifferentialDriveOdometry m_odometry;
    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(20.75));

private static int PULSES_PER_ROTATION = 2048;
private static double WHEEL_DIAMITER = 6.0;

    public DriveSubsystem(WPI_VictorSPX left, WPI_VictorSPX right){
    this.m_left = left;
    this.m_right = right;
  
    // Initializes a duty cycle encoder on DIO pins 0
    this.m_leftEncoder = new Encoder(0,1); 
    this.m_leftEncoder.setDistancePerPulse(Units.inchesToMeters(WHEEL_DIAMITER*Math.PI)/ PULSES_PER_ROTATION);
    this.m_rightEncoder = new Encoder(2,3);
    this.m_rightEncoder.setDistancePerPulse(Units.inchesToMeters(WHEEL_DIAMITER*Math.PI)/ PULSES_PER_ROTATION);

    m_odometry= new DifferentialDriveOdometry(
      new Rotation2d(),
      this.m_leftEncoder.getDistance(),
      this.m_rightEncoder.getDistance(),
      new Pose2d(1.0,2.0, new Rotation2d())
    );
  
    
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

@Override
public void periodic() {
  // Get the rotation of the robot from the gyro.
  var gyroAngle = m_gyro.getRotation2d();

  // Update the pose
  m_pose = m_odometry.update(gyroAngle,
    m_leftEncoder.getDistance(),
    m_rightEncoder.getDistance());
}

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
}
public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
}

public ChassisSpeeds getCurrentSpeeds() {

  var dds = new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(),m_rightEncoder.getRate());
return m_kinematics.toChassisSpeeds(dds);
}

public void drive(ChassisSpeeds chassisSpeeds) {
DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);
m_left.set(wheelSpeeds.leftMetersPerSecond); //TODO convert convert m/s voltage
m_right.set(wheelSpeeds.rightMetersPerSecond);

}

}