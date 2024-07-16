package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems.IntakeSubsystem;


public class IntakeCommands extends Command{
 private final IntakeSubsystem m_IntakeSubsystem;
 public IntakeCommands(IntakeSubsystem intakeSubsystem){
m_IntakeSubsystem = intakeSubsystem;
addRequirements(m_IntakeSubsystem);
 }
 @Override
 public void initialize(){}

 @Override
 public void execute(){}

 @Override
 public void end(boolean inturrupted){}
 
 @Override
 public boolean isFinished(){
    return false;
 }

}
