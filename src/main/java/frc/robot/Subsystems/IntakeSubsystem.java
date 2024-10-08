package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class IntakeSubsystem extends SubsystemBase{
    private final CANSparkMax m_intakeMotor;
    private final CANSparkMax m_rollarMotor;
    private final CANSparkMax m_indexMotor;
    

    public IntakeSubsystem(int intakeID, int rollarID, int indexID){
        m_intakeMotor = new CANSparkMax(intakeID, MotorType.kBrushless);
        m_rollarMotor = new CANSparkMax(rollarID, MotorType.kBrushless);
        m_indexMotor = new CANSparkMax(indexID, MotorType.kBrushless);
        m_rollarMotor.setInverted(true);
        m_indexMotor.setInverted(true);
        m_indexMotor.setIdleMode(IdleMode.kCoast);

        if (Robot.isSimulation()) {
            var sim = REVPhysicsSim.getInstance();
            sim.addSparkMax(m_indexMotor, 1500, 5000);
            sim.addSparkMax(m_intakeMotor, 1500, 5000);
            sim.addSparkMax(m_rollarMotor, 1500, 5000);
        }

    }

    @Override
    public void periodic() {
        updateDashboard();
    }

    @Override
    public void simulationPeriodic() {
        var voltage = RobotController.getBatteryVoltage();
        m_indexMotor.setVoltage(voltage * m_indexMotor.get());
        m_intakeMotor.setVoltage(voltage * m_intakeMotor.get());
        m_rollarMotor.setVoltage(voltage * m_rollarMotor.get());
    }

    void updateDashboard() {
        SmartDashboard.putNumber("Intake Motor", m_intakeMotor.get());
        SmartDashboard.putNumber("Index Motor", m_indexMotor.get());
        SmartDashboard.putNumber("Rollar Motor", m_rollarMotor.get());
    }

    public void startIntake() {
        m_intakeMotor.set(-0.25);
        m_rollarMotor.set(0.25);
        m_indexMotor.set(-0.75);
    }

    public void reverseIntake() {
        m_intakeMotor.set(0.25);
        m_rollarMotor.set(-0.25);
        m_indexMotor.set(-0.5);
    }
    
    public void stopIntake() {
        m_intakeMotor.set(0);
        m_rollarMotor.set(0);
        m_indexMotor.set(0);
    }

    public void indexIn(){
        m_indexMotor.set(0.5);
    };
    public void indexShoot(){
        m_indexMotor.set(-1.0);
    };

    public void indexOut(){
        m_indexMotor.set(-0.5);
    };

    public void indexStop(){
        m_indexMotor.set(0);
    };
}
