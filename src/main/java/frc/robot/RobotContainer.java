// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.cGroup_Intake;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.subsystem_Wrist;
import frc.robot.subsystems.subsystem_Breakers;
import frc.robot.subsystems.subsystem_Indexer;
import frc.robot.subsystems.subsystem_Intake;

import frc.robot.commands.command_MoveWrist;
import frc.robot.commands.command_RunIntake;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final subsystem_Wrist m_Subsystem_Wrist = new subsystem_Wrist();
  private final subsystem_Breakers m_Subsystem_Breakers = new subsystem_Breakers();
  private final subsystem_Intake m_Subsystem_Intake = new subsystem_Intake();
  private final subsystem_Indexer m_Subsystem_Indexer = new subsystem_Indexer();

   private final Joystick haPerator = new Joystick(ControllerConstants.xboxHaperatorID);

  private final JoystickButton intakeTime = 
      new JoystickButton(haPerator, Constants.ControllerConstants.xboxY);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    SmartDashboard.putBoolean("Robot container initialized", true);
    m_Subsystem_Intake.setDefaultCommand(new command_RunIntake(m_Subsystem_Intake, m_Subsystem_Breakers));

    // s_Wrist.setDefaultCommand();
    //  s_Wrist.setDefaultCommand(new command_MoveWrist(s_Wrist,
                                                         
    //                                                       () -> {return haPerator.getRawButtonPressed(Constants.ControllerConstants.xboxX);},
    //                                                       () -> {return haPerator.getRawButtonPressed(Constants.ControllerConstants.xboxY);}));
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

    m_driverController.x().onTrue(new command_MoveWrist(m_Subsystem_Wrist, () -> WristConstants.kWristExtendVal));
    m_driverController.y().onTrue(new command_MoveWrist(m_Subsystem_Wrist, () -> WristConstants.kWristRetractVal));
   
    //new Trigger(m_Subsystem_Intake.isBeamBroken()).onTrue(new command_RunIntake(m_Subsystem_Intake, m_Subsystem_Breakers));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    intakeTime.onTrue(cGroup_Intake.intakeTime(m_Subsystem_Wrist, m_Subsystem_Intake, m_Subsystem_Breakers, m_Subsystem_Indexer));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

    // import frc.robot.Constants.OperatorConstants;
// import frc.robot.commands.Autos;
// import frc.robot.commands.ExampleCommand;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;

// import frc.robot.commands.*;
// import frc.robot.subsystems.*;
// import frc.robot.Constants.ControllerConstants;

// import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
}
