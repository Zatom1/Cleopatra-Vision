// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.Drivebase;
import edu.wpi.first.wpilibj2.command.Command;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final static Drivebase m_drivebase = new Drivebase();
  public final ArcadeDrive m_arcadeDrive = new ArcadeDrive(m_drivebase);
  public final static XboxController xbox0 = new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    //m_drivebase.setDefaultCommand(m_arcadeDrive);
  }

  public static XboxController getXbox0(){
    return xbox0;
  }

  public static double getDriveRightTrigger() {
    return getXbox0().getRightTriggerAxis();
  }
  
  public static boolean getAButton() {
    return getXbox0().getAButtonPressed();
  }

  public static double getDriveLeftTrigger() {
    return getXbox0().getLeftTriggerAxis();
  }

  public static double getDriveSteer() {
    return getXbox0().getLeftX();
  }

  public static double getRightY() {
    return getXbox0().getRightY();
  }
  
  public static double getLeftX() {
    return getXbox0().getLeftY();
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}



}
