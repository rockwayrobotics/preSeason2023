// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;


public class DrivebaseSubsystem extends SubsystemBase {
  private final DifferentialDrive m_drive;

  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  CANSparkMax rightMotorController1;
  CANSparkMax rightMotorController2;
  CANSparkMax leftMotorController1;
  CANSparkMax leftMotorController2;

  private double m_y = 0;
  private double m_x = 0;
  private double m_scale = 1;

  private boolean rotating = false;
  private double rightPow = 0;
  private double leftPow = 0;

  /** Creates a new DrivebaseSubsystem. */
  public DrivebaseSubsystem(
    int leftMotor1, int leftMotor2,
    int rightMotor1, int rightMotor2,
    int leftEncoder1, int leftEncoder2,
    int rightEncoder1, int rightEncoder2
  ) 
  
  /* Create motor controller groups for left and right side of drivebase */
  {

    rightMotorController1 = new CANSparkMax(rightMotor1, MotorType.kBrushed);
    rightMotorController2 = new CANSparkMax(rightMotor2, MotorType.kBrushed);
    leftMotorController1 = new CANSparkMax(leftMotor1, MotorType.kBrushed);
    leftMotorController2 = new CANSparkMax(leftMotor2, MotorType.kBrushed);

    MotorControllerGroup rightDrive = new MotorControllerGroup(
        rightMotorController1, rightMotorController2
    );
    MotorControllerGroup leftDrive = new MotorControllerGroup(
      leftMotorController1, leftMotorController2
    );

    rightDrive.setInverted(true);
    leftDrive.setInverted(true);
    /* Create new DifferentialDrive from the previously created MotorControllerGroups */
    m_drive = new DifferentialDrive(leftDrive, rightDrive);

    m_leftEncoder = new Encoder(leftEncoder1, leftEncoder2);
    m_leftEncoder.setReverseDirection(true);
    m_rightEncoder = new Encoder(rightEncoder1, rightEncoder2);
    
    // when robot goes forward, left encoder spins positive and right encoder spins negative
    m_leftEncoder.setDistancePerPulse(Drive.DISTANCE_PER_ENCODER_PULSE);
    m_rightEncoder.setDistancePerPulse(Drive.DISTANCE_PER_ENCODER_PULSE);
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Sets the speed of the drivebase.
   * @param y Y speed. -1 is full backwards, 1 is full forwards.
   * @param x X speed. -1 is full left, 1 is full right.
   */
  public void set(double x, double y) {
    m_y = y;
    m_x = x;
  }

  /**
   * Sets the scale for the drivebase. Speeds are multiplied by the scale before being sent to the motors.
   * @param scale New scale to multiply speed by.
   */
  public void setScale(double scale) {
    m_scale = scale;
  }

  /**
   * Gets the distance travelled by the left-side wheels of the drivebase since last reset.
   * @return Distance, in inches.
   */
  public double getLDistance() {
    return m_leftEncoder.getDistance();
  }

  public void disable() {
    rightMotorController1.setIdleMode(Drive.DISABLED_MODE);
    rightMotorController2.setIdleMode(Drive.DISABLED_MODE);
    leftMotorController1.setIdleMode(Drive.DISABLED_MODE);
    leftMotorController2.setIdleMode(Drive.DISABLED_MODE);
  }

  public void enable() {
    rightMotorController1.setIdleMode(Drive.ACTIVE_MODE);
    rightMotorController2.setIdleMode(Drive.ACTIVE_MODE);
    leftMotorController1.setIdleMode(Drive.ACTIVE_MODE);
    leftMotorController2.setIdleMode(Drive.ACTIVE_MODE);
  }
  
  /**
   * Gets the distance travelled by the right-side wheels of the drivebase since last reset.
   * @return Distance in inches.
   */
  public double getRDistance() {
    return m_rightEncoder.getDistance();
  }
  
  /**
   * Gets the speed of the left-side wheels of the drivebase.
   * @return Speed in inches / second.
   */
  public double getLRate() {
    return m_leftEncoder.getRate();
  }
  
  /**
   * Gets the speed of the left-side wheels of the drivebase.
   * @return Speed in inches / second.
   */
  public double getRRate() {
    return m_rightEncoder.getRate();
  }

  /**
   * Gets whether the drivebase is currently stopped.
   * @return true if stopped, false if moving.
   */
  public boolean getStopped() {
    return m_leftEncoder.getStopped() && m_rightEncoder.getStopped();
  }

  /** Resets drivebase encoder distances to 0. */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public void stopRobot() {
    m_x = 0;
    m_y = 0;
  }

  public void rotate(String rotate) {
    if(rotate == null) {
      rightPow = 0;
      leftPow = 0;
    } else if(rotate == "left") {
      rotating = true;
      rightPow = 0.3;
      leftPow = -0.3;
    } else if(rotate == "right") {
      rotating = true;
      rightPow = -0.3;
      leftPow = 0.3;
    }
  }

  /* Periodic method that runs once every cycle */
  @Override
  public void periodic() {
    
    // Optional Driving control 2
    //m_drive.arcadeDrive(m_x, m_y);

    // Optional Driving control 3
    m_drive.curvatureDrive((m_x*.8)*m_scale, m_y*m_scale, true);

    // attempting a pure rotating function.
    if(rotating) {
      m_drive.tankDrive(rightPow,leftPow);
      if(rightPow == 0 || leftPow == 0) {
        rotating = false;
      }
    }
  }
}