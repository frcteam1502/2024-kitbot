package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    private final CANSparkMax m_intakeMotor;
    private final CANSparkMax m_rollarMotor;
    private final CANSparkMax m_indexMotor;
    

    public IntakeSubsystem(int intakeID, int rollarID, int indexID){
        m_intakeMotor = new CANSparkMax(intakeID, MotorType.kBrushless);
        m_rollarMotor = new CANSparkMax(rollarID, MotorType.kBrushless);
        m_indexMotor = new CANSparkMax(indexID, MotorType.kBrushless);
    }

    public void startIntake() {
        m_intakeMotor.set(0.25);
        m_rollarMotor.set(0.25);
        m_indexMotor.set(0.5);
    }
    

    public void stopIntake() {
        m_intakeMotor.set(0);
        m_rollarMotor.set(0);
        m_indexMotor.set(0);
    }
}
