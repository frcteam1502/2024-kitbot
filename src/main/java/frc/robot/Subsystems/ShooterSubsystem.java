package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax m_topShooter;
    private final CANSparkMax m_bottemShooter;

    public ShooterSubsystem(int topID, int bottomID) {
        m_topShooter = new CANSparkMax(topID, MotorType.kBrushless);
        m_bottemShooter = new CANSparkMax(bottomID, MotorType.kBrushless);
        
        if (Robot.isSimulation()) {
            var sim = REVPhysicsSim.getInstance();
            sim.addSparkMax(m_topShooter, 1500, 5000);
            sim.addSparkMax(m_bottemShooter, 1500, 5000);
        }
    }

    @Override
    public void periodic() {
        updateDashboard();
    }
    
    @Override
    public void simulationPeriodic() {
        var voltage = RobotController.getBatteryVoltage();
        m_topShooter.setVoltage(voltage * m_topShooter.get());
        m_bottemShooter.setVoltage(voltage * m_bottemShooter.get());        
    }

    void updateDashboard() {
        SmartDashboard.putNumber("Shooter Motor", m_topShooter.get());
    }

    public void shoomp() {
        m_topShooter.set(1);
        m_bottemShooter.set(1);
    }
    public void amp() {
        m_topShooter.set(0.1);
        m_bottemShooter.set(0.1);
    }
    public void intake() {
        m_topShooter.set(-0.2);
        m_bottemShooter.set(-0.2);
    }
    public void stop() {
        m_topShooter.set(0);
        m_bottemShooter.set(0);
    }
}
