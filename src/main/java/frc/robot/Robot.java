// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.autos.ComplexAuto;
import frc.robot.autos.TestAuto;;

/**
 * This is a demo program showing the use of the DifferentialDrive class,
 * specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private final DriveSubsystem m_driveSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;

  private final CommandXboxController driver;
  private final CommandXboxController operator;

  private final WPI_VictorSPX m_FL;
  private final WPI_VictorSPX m_BL;
  private final WPI_VictorSPX m_FR;
  private final WPI_VictorSPX m_BR;

  private final SendableChooser<Command> m_chooser;

  public Robot() {
    // LEFT side
    m_FL = new WPI_VictorSPX(18);
    m_BL = new WPI_VictorSPX(19);
    // RIGHT side
    m_FR = new WPI_VictorSPX(2);
    m_BR = new WPI_VictorSPX(1);

    //m_myRobot = new DifferentialDrive(m_FL, m_FR);

    m_driveSubsystem = new DriveSubsystem(m_FL, m_FR);
    m_intakeSubsystem = new IntakeSubsystem(14, 5, 16);
    m_shooterSubsystem = new ShooterSubsystem(4, 3);
    
    driver = new CommandXboxController(0);
    operator = new CommandXboxController(1);
    
    m_chooser = new SendableChooser<>();
  }

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_FR.setInverted(true);
    m_BR.setInverted(true);

    m_BL.follow(m_FL);
    m_BR.follow(m_FR);

    m_chooser.setDefaultOption("Default Auto", new TestAuto(m_driveSubsystem));
    m_chooser.addOption("Left", new ComplexAuto(m_driveSubsystem, -45));
    m_chooser.addOption("Center", new TestAuto(m_driveSubsystem));
    m_chooser.addOption("Right", new ComplexAuto(m_driveSubsystem, 45));
    SmartDashboard.putData("Auto choices", m_chooser);

    configureBindings();
  }

  private void configureBindings() {
    // = DRIVER =====================

    // Press either bumper to brake
    driver.leftBumper()
      .onTrue(new InstantCommand(() -> m_driveSubsystem.setMaxOutput(0.5)))
      .onFalse(new InstantCommand(() -> m_driveSubsystem.setMaxOutput(1.0)));
    driver.rightBumper()
      .onTrue(new InstantCommand(() -> m_driveSubsystem.setMaxOutput(0.5)))
      .onFalse(new InstantCommand(() -> m_driveSubsystem.setMaxOutput(1.0)));


    // = OPERATOR ===================

    // SHOOMP
    operator.leftBumper()
      .onTrue(new InstantCommand(()->m_shooterSubsystem.shoomp()))
      .onFalse(new InstantCommand(()->m_shooterSubsystem.stop()));

    // AMP CONTROLL
    operator.a()
      .onTrue(new InstantCommand(()->m_shooterSubsystem.amp()))
      .onFalse(new InstantCommand(()->m_shooterSubsystem.stop()));

    // INTAKE
    operator.rightBumper()
      .onTrue(new InstantCommand(()->m_shooterSubsystem.intake()))
      .onFalse(new InstantCommand(()->m_shooterSubsystem.stop()));

    // Set up intake to move note into robot
    operator.y()
      .onTrue(new InstantCommand(()->m_intakeSubsystem.startIntake()))
      .onFalse(new InstantCommand(()->m_intakeSubsystem.stopIntake()));

    operator.back() // try to reverse note
      .onTrue(new InstantCommand(()->m_intakeSubsystem.reverseIntake()))
      .onFalse(new InstantCommand(()->m_intakeSubsystem.stopIntake()));

    // index
    
    operator.b()
      .onTrue(new InstantCommand(()->m_intakeSubsystem.indexIn()))
      .onFalse(new InstantCommand(()->m_intakeSubsystem.indexStop()));

    operator.x()
      .onTrue(new InstantCommand(()->m_intakeSubsystem.indexShoot()))
      .onFalse(new InstantCommand(()->m_intakeSubsystem.indexStop()));
  
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit(){
    var auto = getAutonomousCommand();
    if (Robot.isSimulation()) {
      auto = new ComplexAuto(m_driveSubsystem, -45);
    }
    auto.schedule();
  }

  @Override
  public void teleopPeriodic() {
    // Make Churro move forward/backward/turnLeft/turnRight
    m_driveSubsystem.tankDrive(-driver.getLeftY(), -driver.getRightY());
  }

  @Override
  public void testPeriodic() {
    m_FL.set(0.1);
    m_FR.set(0.1);
    super.testPeriodic();
  }

}