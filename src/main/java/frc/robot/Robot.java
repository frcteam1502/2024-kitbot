// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Subsystems.DriveSubsystem;
//import frc.robot.Subsystems.IntakeSubsystem;
;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {                                                                                                                                                                                                                                                                                                                                     
  private final DriveSubsystem m_driveSubsystem;
  private final DifferentialDrive m_myRobot;
  
  private final Timer waitTimer = new Timer();
  private final Joystick driveControll;
  private final Joystick operator;

  private final WPI_VictorSPX m_FL;
  private final WPI_VictorSPX m_BL;
  private final WPI_VictorSPX m_FR;
  private final WPI_VictorSPX m_BR;

  private final CANSparkMax TopShooter;
  private final CANSparkMax BottemShooter;
  private final CANSparkMax IntakeMotor;
  private final CANSparkMax IndexMotor;
  private final CANSparkMax RollarMotor;
 
  
  double stick_r;
  double stick_l; 


  public Robot() {
    // LEFT side
    m_FL = new WPI_VictorSPX(18);
    m_BL = new WPI_VictorSPX(19);
    // RIGHT side
    m_FR = new WPI_VictorSPX(2);
    m_BR = new WPI_VictorSPX(1);
    TopShooter = new CANSparkMax(4, MotorType.kBrushless);
    BottemShooter = new CANSparkMax(3, MotorType.kBrushless);
    IntakeMotor = new CANSparkMax(14, MotorType.kBrushless);
    IndexMotor = new CANSparkMax(16, MotorType.kBrushless);
    RollarMotor = new CANSparkMax(5, MotorType.kBrushless);
    m_driveSubsystem = new DriveSubsystem(m_FL, m_FR);
    m_myRobot = new DifferentialDrive(m_FL, m_FR);
    driveControll = new Joystick(0);
    operator = new Joystick(1);
  }

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //m_FL.set(ControlMode.PercentOutput, kDefaultPeriod);                                                                                                                        
    //m_FR.set(ControlMode.PercentOutput, kDefaultPeriod);
   
    m_BL.follow(m_FL);
    m_BR.follow(m_FR);
            
  
   // BottemShooter.follow(TopShooter);
    //TopShooter.set(0.7);
    m_FL.setInverted(true);
    m_BL.setInverted(true);
    RollarMotor.setInverted(true);
    IntakeMotor.setInverted(true);


    //m_BL.set(0.5);
    //m_BR.set(0.5);
    //m_FL.set(0.4);
    //m_FR.set(0.4);
  
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }


  @Override
  public void teleopPeriodic() {
    stick_r = driveControll.getRawAxis(5);
    stick_l = driveControll.getRawAxis(1);

    m_FL.set(stick_l*0.8);
    m_FR.set(stick_r*0.82);

    //if (driveControll.getRawButton(1) == true); {
    //m_FL.set(stick_l*0.5);
    //m_FR.set(stick_r*0.52);
      
    
    //  } else if(driveControll.getRawButton(1) == false) {
    //  m_FL.set(stick_l*0.8);
    //  m_FR.set(stick_r*0.82);
    //}

    //Right = 6, Left = 5
    // Make note go SHOOMP
    if(operator.getRawButton(5) == true && driveControll.getRawButton(6) == false){

     // Timer.delay(.75); 
      System.out.println(waitTimer.get() * 10000);
      if(waitTimer.get() * 10000 == 0.0){
        waitTimer.start();
      }

      TopShooter.set(1);
      if(waitTimer.hasElapsed(1.0)){
        BottemShooter.set(1);
        IndexMotor.set(0.5);
      }
      
    }
    //make note come in
    else if(operator.getRawButton(6) == true && driveControll.getRawButton(5) == false){
      TopShooter.set(-0.2);
      BottemShooter.set(-0.2);
      //System.out.println(TopShooter.get());
    }
    else{
      TopShooter.set(0); 
      BottemShooter.set(0);
      IndexMotor.set(0);
      waitTimer.stop();
      waitTimer.reset();
      //System.out.println(TopShooter.get());
    }

    // Make Churro move forward/backward/turnLeft/turnRight

    if(driveControll.getRawButton(1) == true){
    
      m_FL.set(stick_l*0.25);
      //m_FR.set(stick_r*0.27);
      this.m_driveSubsystem.drive(new ChassisSpeeds(stick_l*0.25, 0,0));
    }
    else if(driveControll.getRawButton(1) == false){
       m_FL.set(stick_l*0.8);
      m_FR.set(stick_r*0.82);
    }


    //Set up intake to move note into robot

    if (operator.getRawButton(4) == true) {
      IntakeMotor.set(0.25);
      RollarMotor.set(0.25);
      IndexMotor.set(0.5);
      
    }
    else if (operator.getRawButton(4) == false) {
      IntakeMotor.set(0);
      RollarMotor.set(0);
      IndexMotor.set(0);
    }

    //index 

    if (operator.getRawButton(2) == true) {
      IndexMotor.set(0.5);
    }
    else if (operator.getRawButton(3) == true) {
      IndexMotor.set(-0.5); 
    }
    else{
      IndexMotor.set(0); 
    }

    //AMP CONTROLL

    if (operator.getRawButton(1) == true) {
      TopShooter.set(0.1);
      BottemShooter.set(0.1);
    }
    else if (operator.getRawButton(1) == false) {
      TopShooter.set(0);
      BottemShooter.set(0);   
    }

    //System.out.println("Front Left Speed: " + m_FL.get());
    //System.out.println("Front Right Speed: " + m_FR.get());
    //System.out.println("Back Left Speed: " + m_BL.get());
    //System.out.println("Back Right Speed: " + m_BR.get());
    
    //m_myRobot.tankDrive(stick_l*0.3 stick_r*0.3);
    SmartDashboard.putData(m_myRobot);
  }

  @Override
  public void testPeriodic() {
      
     //m_FL.set(stick_l*0.1);
     //m_FR.set(stick_l*0.1);
     m_FL.set(0.1);
     m_FR.set(0.1);
      super.testPeriodic();
  }

}