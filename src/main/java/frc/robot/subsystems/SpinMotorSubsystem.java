package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class SpinMotorSubsystem extends SubsystemBase{
private double m_speed = 0;

CANSparkMax m_SpinMotor;

public SpinMotorSubsystem(int spinMotor) {
    
m_SpinMotor = new CANSparkMax (spinMotor, MotorType.kBrushless);

}

@Override
public void periodic(){
    m_SpinMotor.set(m_speed);
}

public void spin(double speed){
    m_speed = speed;
}






}
