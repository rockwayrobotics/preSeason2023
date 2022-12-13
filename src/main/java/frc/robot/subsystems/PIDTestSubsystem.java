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
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, kVelocityTarget;

    // encoder using rev's libraries 
    private RelativeEncoder m_pidEncoder; 

    // Make a PID Controller using rev's libraries
    private SparkMaxPIDController m_pidcontroller;

    public PIDTestSubsystem(int PIDMotor){

    // New PIDTest motor 
    m_pidtest = new CANSparkMax(PIDMotor, MotorType.kBrushless);
    
    m_pidEncoder = m_pidtest.getEncoder();
    m_pidcontroller = m_pidtest.getPIDController();


    // PID coefficients; a few pulled from SysID to start with 
    // Also used https://www.chiefdelphi.com/t/spark-max-w-neos-pid-not-getting-to-setpoint/381728/2 for kFF

    kP = 0.0029014; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 2.439024390243902e-4;
    kMaxOutput = 1; 
    kMinOutput = 0;
    kVelocityTarget = 4100;

    // Set PID Coeffiecients

    m_pidcontroller.setP(kP);
    m_pidcontroller.setI(kI);
    m_pidcontroller.setD(kD);
    m_pidcontroller.setIZone(kIz);
    m_pidcontroller.setFF(kFF);
    m_pidcontroller.setOutputRange(kMinOutput, kMaxOutput);
    
    // Display PID Coeffecicients on SmartDashboard

    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Motor Target RPM", kVelocityTarget);
  }

  public void spinPIDTestSpeed () {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double setpoint = SmartDashboard.getNumber("Motor Target RPM", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidcontroller.setP(p); kP = p; }
    if((i != kI)) { m_pidcontroller.setI(i); kI = i; }
    if((d != kD)) { m_pidcontroller.setD(d); kD = d; }
    if((iz != kIz)) { m_pidcontroller.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidcontroller.setFF(ff); kFF = ff; }

    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidcontroller.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    if((setpoint != kVelocityTarget)) { 
      // m_pidcontroller.setReference(kVelocityTarget, CANSparkMax.ControlType.kVelocity); 
      kVelocityTarget = setpoint;
    }

    // m_pidcontroller.setReference(kVelocityTarget, CANSparkMax.ControlType.kVelocity);
    // m_pidcontroller.setFeedbackDevice(m_pidEncoder);
  }

  /**
   * Spins the motor at a specified power level.
   * @param PIDPow Speed to spin the wheel. -1 is full backwards, 1 is full forwards.
   */
  public void spin(double PIDPow) {
    m_pidtestPow = PIDPow;
  }

  public void pidspin() {
    m_pidcontroller.setReference(kVelocityTarget, CANSparkMax.ControlType.kVelocity);
  }

  public void pidstop(){
    m_pidcontroller.setReference(0, CANSparkMax.ControlType.kVelocity);
  }

  public void stop(){
    m_pidtestPow = 0;
  }

  public double getVelocity(){
    return m_pidEncoder.getVelocity();
  }


  @Override
  public void periodic(){
    SmartDashboard.putNumber("Current Motor RPM", m_pidEncoder.getVelocity());
    spinPIDTestSpeed();
    //m_pidtest.set(m_pidtestPow);
  }
}
