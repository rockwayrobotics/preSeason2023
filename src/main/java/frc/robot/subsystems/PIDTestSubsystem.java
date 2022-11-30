package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PIDTestSubsystem extends SubsystemBase{
    CANSparkMax m_pidtest;

    private double m_pidtestPow = 0; 
    private double kP, kI, kD, kVelocityTarget;

    // encoder using rev's libraries 
    private RelativeEncoder m_pidEncoder; 

    // Make a PID Controller using rev's libraries
    private SparkMaxPIDController m_pidcontroller;

    public PIDTestSubsystem(
        int PIDMotor
    ){
    // New PIDTest motor 
    m_pidtest = new CANSparkMax(PIDMotor, MotorType.kBrushless);

    // PID coefficients; a few pulled from SysID to start with 
    kP = 0.0029014;
    kI = 0;
    kD = 0.023716; 
    kVelocityTarget = 4100;

    // Set PID Coeffiecients
    m_pidcontroller.setP(kI);
    m_pidcontroller.setI(kI);
    m_pidcontroller.setD(kD);
    
    // Display PID Coeffecicients on SmartDashboard

    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("Motor Target RPM", kVelocityTarget);
    }

  public void spinPIDTestSpeed () {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double setpoint = SmartDashboard.getNumber("Motor Target RPM", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidcontroller.setP(p); kP = p; }
    if((i != kI)) { m_pidcontroller.setI(i); kI = i; }
    if((d != kD)) { m_pidcontroller.setD(d); kD = d; }
    if((setpoint != kVelocityTarget)) { m_pidcontroller.setReference(kVelocityTarget, CANSparkMax.ControlType.kVoltage); kVelocityTarget = setpoint;}

    m_pidcontroller.setReference(kVelocityTarget, CANSparkMax.ControlType.kVelocity);
  }

  /**
   * Spins the motor at a specified power level.
   * @param PIDPow Speed to spin the wheel. -1 is full backwards, 1 is full forwards.
   */
  public void spin(double PIDPow) {
    m_pidtestPow = PIDPow;
  }

  public void stop(){
    m_pidcontroller.setReference(0, CANSparkMax.ControlType.kVoltage);
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("Current Motor RPM", m_pidEncoder.getVelocity());
    m_pidtest.set(m_pidtestPow);
  }
}
