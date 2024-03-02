// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.command_DriveTeleop;
import frc.robot.subsystems.subsystem_DriveTrain;
import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.commands.ExampleCommand;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  /* Controllers */  
  private final Joystick driver = new Joystick(ControllerConstants.xboxDriveID);
  
  /* Drive Controls */
  private final int translationAxis = ControllerConstants.xboxLYAxis;
  private final int strafeAxis = ControllerConstants.xboxLXAxis;
  private final int rotationAxis = ControllerConstants.xboxRXAxis;

  private final int zeroGyroButton = ControllerConstants.xboxMenu;
  private final int changeLinearThrottleButton = ControllerConstants.xboxLeftJoyPress;
  private final int changeAngularThrottleButton = ControllerConstants.xboxRightJoyPress;
  private final int parkButton = ControllerConstants.xboxView;

  private final int quasistaticDriveForwardButton = ControllerConstants.xboxA;
  private final int quasistaticDriveBackButton = ControllerConstants.xboxX;
  private final int dynamicDriveForwardButton = ControllerConstants.xboxB;
  private final int dynamicDriveBackButton = ControllerConstants.xboxY;

  // private final int quasistaticAngleForwardButton = ControllerConstants.xboxA;
  // private final int dynamicAngleForwardButton = ControllerConstants.xboxB;
  // private final int quasistaticAngleBackButton = ControllerConstants.xboxX;
  // private final int dynamicAngleBackButton = ControllerConstants.xboxY;


  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, zeroGyroButton);
  private final JoystickButton changeLinearThrottle = new JoystickButton(driver, changeLinearThrottleButton);
  private final JoystickButton changeAngularThrottle = new JoystickButton(driver, changeAngularThrottleButton);
  private final JoystickButton park = new JoystickButton(driver, parkButton);

  private final JoystickButton quasistaticDriveF = new JoystickButton(driver, quasistaticDriveForwardButton);
  private final JoystickButton dynamicDriveF = new JoystickButton(driver, dynamicDriveForwardButton);
  private final JoystickButton quasistaticDriveB = new JoystickButton(driver, quasistaticDriveBackButton);
  private final JoystickButton dynamicDriveB = new JoystickButton(driver, dynamicDriveBackButton);
  
  // private final JoystickButton quasistaticAngleF = new JoystickButton(driver, quasistaticAngleForwardButton);
  // private final JoystickButton dynamicAngleF = new JoystickButton(driver, dynamicAngleForwardButton);
  // private final JoystickButton quasistaticAngleB = new JoystickButton(driver, quasistaticAngleBackButton);
  // private final JoystickButton dynamicAngleB = new JoystickButton(driver, dynamicAngleBackButton);

  /* Subsystems */
  private final subsystem_DriveTrain m_DriveTrain = new subsystem_DriveTrain();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_DriveTrain.setDefaultCommand(new command_DriveTeleop(m_DriveTrain, 
                                                          () -> {return -driver.getRawAxis(translationAxis);},
                                                          () -> {return -driver.getRawAxis(strafeAxis);},
                                                          // () -> {return -driver.getRawAxis(rotationAxis);},
                                                          // () -> {return 0.0;},
                                                          // () -> {return 0.0;},
                                                          () -> {return 0.101;},
                                                          () -> {return driver.getRawButtonPressed(ControllerConstants.xboxLB);},
                                                          () -> {return driver.getRawButtonPressed(ControllerConstants.xboxRB);},
                                                          () -> {return SwerveConstants.isFieldRelative;},
                                                          () -> {return SwerveConstants.isOpenLoop;},
                                                          () -> {return false;}));
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    // zeroGyro.onTrue(new InstantCommand(() -> m_DriveTrain.zeroGyroCommand()));
    // changeThrottle.onTrue(new InstantCommand(() -> m_DriveTrain.toggleThrottleCommand()));
    zeroGyro.onTrue(m_DriveTrain.zeroGyroInstantCommand());
    changeLinearThrottle.onTrue(m_DriveTrain.toggleLinearThrottleCommand());
    changeAngularThrottle.onTrue(m_DriveTrain.toggleAngularThrottleCommand());
    park.onTrue(m_DriveTrain.toggleParkCommand());

    quasistaticDriveF.whileTrue(m_DriveTrain.driveQuasistatic(Direction.kForward));
    quasistaticDriveB.whileTrue(m_DriveTrain.driveQuasistatic(Direction.kReverse));
    dynamicDriveF.whileTrue(m_DriveTrain.driveDynamic(Direction.kForward));
    dynamicDriveB.whileTrue(m_DriveTrain.driveDynamic(Direction.kReverse));

    // quasistaticAngleF.onFalse(m_DriveTrain.zeroAngleMotors());
    // quasistaticAngleB.onFalse(m_DriveTrain.zeroAngleMotors());
    // dynamicAngleF.onFalse(m_DriveTrain.zeroAngleMotors());
    // dynamicAngleB.onFalse(m_DriveTrain.zeroAngleMotors());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_DriveTrain);
    return Autos.choreoCommand(m_DriveTrain, "NewPath");
  }
}
