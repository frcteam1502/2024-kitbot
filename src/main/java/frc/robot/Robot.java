// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
//import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.simulation.PWMSim;


/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {                                                                                                                                                                                                                                                                                                                                     
  private DifferentialDrive m_myRobot;
  private Joystick driveControll;
  public  double stick_r;
  public double stick_l; 

  private final WPI_VictorSPX m_FL = new WPI_VictorSPX(18);
  private final WPI_VictorSPX m_BL = new WPI_VictorSPX(19);

  private final WPI_VictorSPX m_FR = new WPI_VictorSPX(2);
  private final WPI_VictorSPX m_BR = new WPI_VictorSPX(1);

  //Shooter Motors: 4 = top, 3 = bottom
  private final CANSparkMax TopShooter = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax BottemShooter = new CANSparkMax(3, MotorType.kBrushless);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //m_FL.set(ControlMode.PercentOutput, kDefaultPeriod);
    //m_FR.set(ControlMode.PercentOutput, kDefaultPeriod);
    m_BL.follow(m_FL);
    m_BR.follow(m_FR);
    
  
    //BottemShooter.follow(TopShooter);
    //TopShooter.set(0.7);
    m_FL.setInverted(true);
    m_BL.setInverted(true);

    m_myRobot = new DifferentialDrive(m_FL, m_FR);
    driveControll = new Joystick(0);
    //m_BL.set(0.5);
    //m_BR.set(0.5);
    //m_FL.set(0.4);
    //m_FR.set(0.4);
  
  }

  @Override
  public void teleopPeriodic() {
    stick_r = driveControll.getRawAxis(5);
    stick_l = driveControll.getRawAxis(1);
    
    //Right = 6, Left = 5
    if(driveControll.getRawButton(5) == true && driveControll.getRawButton(6) == false){
     //BottemShooter.follow(TopShooter) = false;
      TopShooter.set(1.0);

      //BottemShooter.follow(TopShooter) = true;
      //TopShooter.set(1.0);4
      
      //BottemShooter.set(0.4);
      //System.out.println(TopShooter.get());
    }
    else if(driveControll.getRawButton(6) == true && driveControll.getRawButton(5) == false){
      TopShooter.set(-0.2);
      //System.out.println(TopShooter.get());
    }
    else{
      TopShooter.set(0); 
      //System.out.println(TopShooter.get());
    }


    //System.out.println("Front Left Speed: " + m_FL.get());
    //System.out.println("Front Right Speed: " + m_FR.get());
    //System.out.println("Back Left Speed: " + m_BL.get());
    //System.out.println("Back Right Speed: " + m_BR.get());
    
    m_myRobot.tankDrive(stick_l, stick_r);

  }
}
