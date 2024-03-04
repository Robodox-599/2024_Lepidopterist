// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.subsystem_ShooterWheels;
import frc.robot.subsystems.subsystem_ShooterWrist;
import frc.robot.subsystems.subsystem_LED;
import frc.robot.subsystems.subsystem_DriveTrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.command_AutoSpeaker;
import frc.robot.commands.command_ManualWrist;
import frc.robot.commands.command_ToSetWrist;
import frc.robot.commands.command_ToFlywheelSpeed;
import frc.robot.commands.command_cGroup_shooter;
// import frc.robot.commands.enable;
import frc.robot.commands.isReady;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final subsystem_ShooterWheels m_flywheels = new subsystem_ShooterWheels();
  private final subsystem_ShooterWrist m_wrist = new subsystem_ShooterWrist();
  private final subsystem_LED m_LED = new subsystem_LED();
  private final subsystem_DriveTrain m_drivetrain = new subsystem_DriveTrain();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    // m_wrist.setDefaultCommand(new command_ManualWrist(m_wrist,
    //  () -> -m_driverController.getRawAxis(Constants.OperatorConstants.Left_y_axis)));
    configureBindings();
    m_LED.setDefaultCommand(new isReady(m_flywheels, m_wrist, m_LED));
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
    // // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
      // m_driverController.b().onTrue(command_cGroup_shooter.armAmpCommand(m_flywheels, m_wrist, m_LED, m_drivetrain))
                                      ;
      // b for arm amp

      // m_driverController.a().whileTrue(command_cGroup_shooter.shoot(m_flywheels, m_wrist, m_LED, m_drivetrain)) ;
    // a for shoot (generic)

    m_driverController.a().onFalse(command_cGroup_shooter.stow(m_flywheels, m_wrist, m_LED, m_drivetrain));
// y is stow
      //x speaker shoot ready button  
      m_driverController.a().whileTrue(command_cGroup_shooter.armSpeakerCommand(m_flywheels, m_wrist, m_LED, m_drivetrain));
      //make a change to the to amp writst commmand and add a paramet for stow
      m_driverController.b().whileTrue(command_cGroup_shooter.shoot(m_flywheels, m_wrist, m_LED, m_drivetrain));
      m_driverController.b().onFalse(m_LED.set_stateCommand("stow"));

      // m_driverController.x().toggleOnTrue(Commands.startEnd(m_LED::setGreenLED, m_LED::setStandbyLED, m_LED));


      m_driverController.x().whileTrue(command_cGroup_shooter.sourceIntake(m_flywheels, m_wrist));
      m_driverController.x().onFalse(command_cGroup_shooter.stow(m_flywheels, m_wrist, m_LED, m_drivetrain));

      m_driverController.y().whileTrue(command_cGroup_shooter.scoreAmp(m_flywheels, m_wrist));
      m_driverController.y().onFalse(command_cGroup_shooter.stow(m_flywheels, m_wrist, m_LED, m_drivetrain));
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
}
