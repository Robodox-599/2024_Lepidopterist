// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.cGroup_Intake;
// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.command_DriveTeleop;
import frc.robot.subsystems.subsystem_DriveTrain;
import frc.robot.subsystems.subsystem_Indexer;
import frc.robot.subsystems.subsystem_Intake;
import frc.robot.subsystems.subsystem_Vision;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  
  /* Subsystems */
  private final subsystem_DriveTrain m_DriveTrain = new subsystem_DriveTrain();
  private final subsystem_Vision m_Vision = new subsystem_Vision();
  private final subsystem_Intake m_Intake = new subsystem_Intake();
  private final subsystem_Indexer m_Indexer = new subsystem_Indexer();
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  /* Controllers */  
  private final Joystick driver = new Joystick(ControllerConstants.xboxDriveID);
  
  /* Drive Controls */
  private final int translationAxis = ControllerConstants.xboxLYAxis;
  private final int strafeAxis = ControllerConstants.xboxLXAxis;
  private final int rotationAxis = ControllerConstants.xboxRXAxis;

  private final int zeroGyroButton = ControllerConstants.xboxY;
  private final int changeLinearThrottleButton = ControllerConstants.xboxLeftJoyPress;
  private final int changeAngularThrottleButton = ControllerConstants.xboxRightJoyPress;
  private final int extendButton = ControllerConstants.xboxA;
  private final int retractButton = ControllerConstants.xboxB;
  private final int intakeButton = ControllerConstants.xboxX;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, zeroGyroButton);
  private final JoystickButton changeLinearThrottle = new JoystickButton(driver, changeLinearThrottleButton);
  private final JoystickButton changeAngularThrottle = new JoystickButton(driver, changeAngularThrottleButton);
  private final JoystickButton intake = new JoystickButton(driver, intakeButton);
  // private final JoystickButton extend = new JoystickButton(driver, extendButton);
  // private final JoystickButton retract = new JoystickButton(driver, retractButton);

  /* Sendable Chooser */
  private final SendableChooser<Command> m_Chooser = AutoBuilder.buildAutoChooser();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    NamedCommands.registerCommand("Intake Note", cGroup_Intake.runIntakeSequence(m_Intake, m_Indexer));

    m_Chooser.addOption("Rotate 90", Autos.pathPlannerCommand(m_DriveTrain, "90DegAuto"));
    m_Chooser.addOption("Straight", Autos.pathPlannerCommand(m_DriveTrain, "StraightAuto"));
    m_Chooser.addOption("PathPlanner", Autos.pathPlannerCommand(m_DriveTrain, "SAuto"));

    m_DriveTrain.setDefaultCommand(new command_DriveTeleop(m_DriveTrain, m_Vision,
                                                          () -> {return -driver.getRawAxis(translationAxis);},
                                                          () -> {return -driver.getRawAxis(strafeAxis);},
                                                          () -> {return -driver.getRawAxis(rotationAxis);},
                                                          () -> {return driver.getRawButtonPressed(ControllerConstants.xboxLB);},
                                                          () -> {return driver.getRawButtonPressed(ControllerConstants.xboxRB);},
                                                          () -> {return driver.getRawButtonPressed(extendButton);},
                                                          () -> {return driver.getRawButtonPressed(retractButton);},
                                                          () -> {return SwerveConstants.isFieldRelative;},
                                                          () -> {return SwerveConstants.isOpenLoop;}));

    configureBindings();

    SmartDashboard.putData("Auto Chooser", m_Chooser);
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
    intake.onTrue(cGroup_Intake.runIntakeSequence(m_Intake, m_Indexer));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_DriveTrain);
    // return Autos.choreoCommand(m_DriveTrain, "NewPath");
    // return Autos.pathPlannerCommand(m_DriveTrain, "StraightAuto");
    return m_Chooser.getSelected();
  }
}
