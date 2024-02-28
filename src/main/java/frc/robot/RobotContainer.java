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
import frc.robot.subsystems.subsystem_Climb;
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
  private final subsystem_Climb m_Climb = new subsystem_Climb();
  private final subsystem_Intake m_Intake = new subsystem_Intake();
  private final subsystem_Indexer m_Indexer = new subsystem_Indexer();
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  /* Controllers */  
  private final CommandXboxController m_driverController = 
  new CommandXboxController(ControllerConstants.xboxDriveID);
  private final CommandXboxController m_operatorController = 
  new CommandXboxController(ControllerConstants.xboxOperatorID);



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
                                                          () -> {return -m_driverController.getLeftY();},
                                                          () -> {return -m_driverController.getLeftX();},
                                                          () -> {return -m_driverController.getRightX();},
                                                          () -> {return SwerveConstants.isFieldRelative;},
                                                          () -> {return SwerveConstants.isOpenLoop;}));
                                                          
    m_Climb.setDefaultCommand(m_Climb.setSetpointCommand(() -> {return m_operatorController.;},
                                                          () -> {return m_operatorController;},
                                                          () -> {return m_DriveTrain.getRoll();}));
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
    m_driverController.y().onTrue(m_DriveTrain.zeroGyroInstantCommand());
    m_driverController.leftStick().onTrue(m_DriveTrain.toggleLinearThrottleCommand());
    m_driverController.rightStick().onTrue(m_DriveTrain.toggleAngularThrottleCommand());
    m_driverController.leftBumper().onTrue(cGroup_Intake.runIntakeSequence(m_Intake, m_Indexer));

    m_operatorController.leftTrigger().onTrue();
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
