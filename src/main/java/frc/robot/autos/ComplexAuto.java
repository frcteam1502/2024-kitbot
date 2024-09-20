package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.DriveSubsystem;

public class ComplexAuto extends SequentialCommandGroup {

    public ComplexAuto(DriveSubsystem m_drive, int turn) {

        addCommands(
                new TestAuto(m_drive),
                new TurnAuto(m_drive, turn),
                new TestAuto(m_drive));

    }
}
