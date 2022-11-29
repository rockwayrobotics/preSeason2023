package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIDTestSubsystem extends SubsystemBase{
    CANSparkMax m_pidtest;

    private double m_pidtestPow = 0; 

    private double kP, kI, kD;


    public PIDTestSubsystem(
        int PIDMotor
    ){
    m_pidtest = new CANSparkMax(PIDMotor, MotorType.kBrushless);
    }
  public void spinPIDTestSpeed () {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    // Dummy implementation until proper grasp of how these variables affect the motor
    if((p != kP)) { kP = p; }
    if((i != kI)) { kI = i; }
    if((d != kD)) { kD = d; }
  }

  
  /**
   * Spins the index wheel at a specified power level.
   * @param indexPow Speed to spin the wheel. -1 is full backwards, 1 is full forwards.
   */
  public void spin(double PIDPow) {
   m_pidtestPow = PIDPow;
   m_pidtest.set(m_pidtestPow);
  }
}
