package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

public class DriveSubsystem extends SubsystemBase {

  private class Gyro {
    Rotation2d m_rotation = new Rotation2d();
    public Gyro(int id){
    }
    
    public Rotation2d getRotation2d() {
      return m_rotation;
    }

    public int getAngle() {
      return 0;
    }
  }
  public static double TRACK_WIDTH = 23;
  public static double WHEEL_DIAMETER = 6.0;
  public static int PULSES_PER_ROTATION = 360;

  private final WPI_VictorSPX m_leftMotor;
  private final WPI_VictorSPX m_rightMotor;
  private final Gyro m_gyro; // TODO: install pigeon!
  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  private Pose2d m_pose;
  private final DifferentialDriveOdometry m_odometry;
  private final DifferentialDrive m_drive;
  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
      Units.inchesToMeters(TRACK_WIDTH));

  public DriveSubsystem(WPI_VictorSPX left, WPI_VictorSPX right) {
    super();

    m_gyro = new Gyro(0);

    this.m_leftMotor = left;
    this.m_rightMotor = right;

    // Initializes a duty cycle encoder on DIO pins 0
    this.m_leftEncoder = new Encoder(2, 3, true);
    this.m_leftEncoder.reset();
    this.m_leftEncoder.setDistancePerPulse(Units.inchesToMeters(WHEEL_DIAMETER * Math.PI) / PULSES_PER_ROTATION);
    this.m_rightEncoder = new Encoder(0, 1, false);
    this.m_rightEncoder.reset();
    this.m_rightEncoder.setDistancePerPulse(Units.inchesToMeters(WHEEL_DIAMETER * Math.PI) / PULSES_PER_ROTATION);
    m_drive = new DifferentialDrive(this.m_leftMotor, this.m_rightMotor);
    m_odometry = new DifferentialDriveOdometry(
        new Rotation2d(),
        this.getLeftDistance(),
        this.getRightDistance(),
        new Pose2d(0, 0, new Rotation2d()));

    AutoBuilder.configureRamsete(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getCurrentSpeeds, // Current ChassisSpeeds supplier
        this::drive, // Method that will drive the robot given ChassisSpeeds
        new ReplanningConfig(), // Default path replanning config. See the API for the options here
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
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

  double m_simLeftDistance;
  double m_simRightDistance;
  double m_simLeftSpeed;
  double m_simRightSpeed;

  private double getLeftDistance() {
    if (Robot.isReal()) {
      return m_leftEncoder.getDistance();
    } else {
      return m_simLeftDistance;
    }
  }
  private double getRightDistance() {
    if (Robot.isReal()) {
      return m_rightEncoder.getDistance();
    } else {
      return m_simRightDistance;
    }
  }

  @Override
  public void simulationPeriodic() {
    var speedToDistance = .02 /*ms*/ * Units.inchesToMeters(WHEEL_DIAMETER * Math.PI);
    var speedLeft = m_simLeftSpeed * 5.0; // assum 1.0 = 5 ms
    var speedRight = m_simRightSpeed * 5.0; //m_rightMotor.get();
    m_simLeftDistance += speedLeft * speedToDistance;
    m_simRightDistance += speedRight * speedToDistance;
  }
  void updatePose() {
    Twist2d twist = m_kinematics.toTwist2d(getLeftDistance(), getRightDistance());
    m_gyro.m_rotation = Rotation2d.fromRadians(twist.dtheta);
    m_pose = m_odometry.update(m_gyro.getRotation2d(), getLeftDistance(), getRightDistance());
  }

  void updateDashboard() {
    SmartDashboard.putData(m_drive);
    SmartDashboard.putNumber("Gyro Degrees", m_gyro.getAngle());
    SmartDashboard.putNumber("Left count", m_leftEncoder.get());
    SmartDashboard.putNumber("Left Distance", getLeftDistance());
    SmartDashboard.putNumber("Left Velocity", m_leftEncoder.getRate());
    SmartDashboard.putNumber("Right Distance", getRightDistance());
    SmartDashboard.putNumber("Right Velocity", m_rightEncoder.getRate());
    SmartDashboard.putNumber("rotation", m_pose.getRotation().getDegrees());
  }

 public Translation2d getTranslation(){
  return new Translation2d(getLeftDistance(), getRightDistance());
 }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    m_simLeftDistance = 0;
    m_simRightDistance = 0;
    m_gyro.m_rotation = new Rotation2d();
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_odometry.resetPosition(m_gyro.getRotation2d(), getLeftDistance(), getRightDistance(), pose);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    var dds = new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
    return m_kinematics.toChassisSpeeds(dds);
  }

  double m_speedGain = 1.0;
  public void setMaxOutput(double ratio) {
    m_speedGain = ratio;
  }
  public void drive(ChassisSpeeds chassisSpeeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);
    m_leftMotor.set(wheelSpeeds.leftMetersPerSecond); // TODO convert convert m/s voltage
    m_rightMotor.set(wheelSpeeds.rightMetersPerSecond);
  }

  // public void tankDrive(DifferentialDriveWheelSpeeds wheelSpeeds) {
  //   tankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  // }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    leftSpeed *= m_speedGain;
    rightSpeed *= m_speedGain;
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }
  public void autoDrive(double leftSpeed, double rightSpeed) {
    m_simLeftSpeed = leftSpeed;
    m_simRightSpeed = rightSpeed;
    m_leftMotor.set(leftSpeed); // TODO convert convert m/s voltage
    m_rightMotor.set(rightSpeed);
    m_drive.feed();
  }
  
  public void autoStop() {
    autoDrive(0,0); //WARNING: will continue to roll
  }
}