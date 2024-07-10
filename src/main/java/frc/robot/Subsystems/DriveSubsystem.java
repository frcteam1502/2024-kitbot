package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

public class DriveSubsystem extends SubsystemBase {

  private static double TRACK_WIDTH = 20.75;
  private static double WHEEL_DIAMETER = 6.0;
  private static int PULSES_PER_ROTATION = 2048;
   
  private final WPI_VictorSPX m_leftMotor;
  private final WPI_VictorSPX m_rightMotor;
  private final Pigeon2 m_gyro; // TODO: install pigeon!
  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;
  
  private Pose2d m_pose;
  private final DifferentialDriveOdometry m_odometry;
  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(TRACK_WIDTH));

  public DriveSubsystem(WPI_VictorSPX left, WPI_VictorSPX right) {
    super();
    
    m_gyro = new Pigeon2(0);
  
    this.m_leftMotor = left;
    this.m_rightMotor = right;
  
    // Initializes a duty cycle encoder on DIO pins 0
    this.m_leftEncoder = new Encoder(2,3,false, EncodingType.k4X);
    this.m_leftEncoder.reset();
    this.m_leftEncoder.setDistancePerPulse(Units.inchesToMeters(WHEEL_DIAMETER * Math.PI) / PULSES_PER_ROTATION);
    this.m_rightEncoder = new Encoder(0,1,false, EncodingType.k4X);
    this.m_rightEncoder.reset();
    this.m_rightEncoder.setDistancePerPulse(Units.inchesToMeters(WHEEL_DIAMETER * Math.PI) / PULSES_PER_ROTATION);

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
    updatePose();
    updateDashboard();
  }
  
  void updatePose() {
    m_pose = m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());  
  }

  void updateDashboard() {
    SmartDashboard.putNumber("Gyro Degrees", m_gyro.getAngle());
    SmartDashboard.putNumber("Left count", m_leftEncoder.get());
    SmartDashboard.putNumber("Left Distance", m_leftEncoder.getDistance());
    SmartDashboard.putNumber("Left Velocity", m_leftEncoder.getRate());
    SmartDashboard.putNumber("Right Distance", m_rightEncoder.getDistance());
    SmartDashboard.putNumber("Right Velocity", m_rightEncoder.getRate());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    var dds = new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
    return m_kinematics.toChassisSpeeds(dds); 
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);
    m_leftMotor.set(wheelSpeeds.leftMetersPerSecond); //TODO convert convert m/s voltage
    m_rightMotor.set(wheelSpeeds.rightMetersPerSecond);
  }

}