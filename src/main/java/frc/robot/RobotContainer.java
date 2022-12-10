// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.CAN;
import frc.robot.Constants.Controllers;
import frc.robot.Constants.Digital;

import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.PIDTestSubsystem;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.button.Button;

//import java.util.Map;

//import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;

import frc.robot.commands.AutonomousCmdList;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  /* Contstructor for subsystems */
  private DrivebaseSubsystem m_drivebase = new DrivebaseSubsystem(
    CAN.LEFT_MOTOR_1, CAN.LEFT_MOTOR_2,
    CAN.RIGHT_MOTOR_1, CAN.RIGHT_MOTOR_2,
    Digital.LEFT_ENCODER_1, Digital.LEFT_ENCODER_2,
    Digital.RIGHT_ENCODER_1, Digital.RIGHT_ENCODER_2
  );

  private PIDTestSubsystem m_pidtestsubsystem = new PIDTestSubsystem(CAN.PID_MOTOR);
  

  private XboxController m_xboxController = new XboxController(Controllers.XBOX);

  ShuffleboardTab tab = Shuffleboard.getTab("Speeds");
  private NetworkTableEntry autoDistance =
        tab.add("Auto Distance", 50)
            .getEntry();
  private NetworkTableEntry autoSpeed =
        tab.add("Auto Speed", 0.5)
            .getEntry();

  public final Command m_autoCommand = new AutonomousCmdList(m_drivebase, autoDistance, autoSpeed); //pass in drivebase here
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    m_drivebase.setDefaultCommand(
      new RunCommand(
        () -> m_drivebase.set(m_xboxController.getLeftX(), -m_xboxController.getLeftY()),
        m_drivebase
      )
    );  // Runs drivebase off left stick
  
    new JoystickButton(m_xboxController, XboxController.Button.kLeftBumper.value)
    .whenPressed(() -> m_drivebase.setScale(0.5))
    .whenReleased(() -> m_drivebase.setScale(1));  // Sets drivebase to half speed, for more precise and slow movement (likely going to be used inside hangar)
    
    new JoystickButton(m_xboxController, XboxController.Button.kA.value)
    .whenPressed(() -> m_pidtestsubsystem.spin(0.6))
    .whenReleased(() -> m_pidtestsubsystem.spin(0));  

    new JoystickButton(m_xboxController, XboxController.Button.kB.value)
    .whenPressed(() -> m_pidtestsubsystem.pidspin());

    new JoystickButton(m_xboxController, XboxController.Button.kY.value)
    .whenPressed(() -> m_pidtestsubsystem.pidstop());

    
    /* 
    This is example code for more robot functions

    new JoystickButton(m_xboxController, XboxController.Button.kA.value) // Manually jogs indexer wheel away from the flywheel
    .whenPressed(new InstantCommand(() -> m_shooter.spinIndex(0.3), m_shooter))
    .whenReleased(new InstantCommand(() -> m_shooter.spinIndex(0), m_shooter));

    new JoystickButton(m_xboxController, XboxController.Button.kB.value)  // Retracts intake
    .whenPressed(() -> m_intake.retract());

    */

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Uses m_autoCommand which was created earlier
    return m_autoCommand;
  }

  public void onDisable() {
    m_drivebase.disable();
  }
  
  public void onEnable() {
    m_drivebase.enable();
  }
}
