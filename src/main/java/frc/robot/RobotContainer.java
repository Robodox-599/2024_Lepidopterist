// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.DRIVE_STATE;
// import frc.robot.commands.Autos;
// import frc.robot.commands.ExampleCommand;
// import frc.robot.subsystems.subsystem_ShooterWheels;
import frc.robot.subsystems.subsystem_ShooterWrist;
import frc.robot.subsystems.subsystem_Vision;
import frc.robot.subsystems.subsystem_LED;
import frc.robot.subsystems.subsystem_DriveTrain;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.command_AutoSpeakerWrist;
import frc.robot.commands.command_DriveTeleop;
import frc.robot.commands.command_ManualWrist;
import frc.robot.commands.command_ToAmpWrist;
// import frc.robot.commands.command_ToFlywheelSpeed;
// import frc.robot.commands.enable;
import frc.robot.commands.command_WaitUntilReady;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final subsystem_ShooterWheels m_flywheels = new subsystem_ShooterWheels();
  private final subsystem_ShooterWrist m_wrist = new subsystem_ShooterWrist();
  private final subsystem_LED m_LED = new subsystem_LED();
  private final subsystem_DriveTrain m_drive = new subsystem_DriveTrain();
  private final subsystem_Vision m_Vision = new subsystem_Vision();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
  new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    // m_wrist.setDefaultCommand(new command_ManualWrist(m_wrist,
    //  () -> -m_driverController.getRawAxis(Constants.OperatorConstants.Left_y_axis)));
    configureBindings();
    // m_LED.setDefaultCommand(new isReadyLEDs(/*m_flywheels,*/ m_wrist, m_LED));
    m_drive.setDefaultCommand(new command_DriveTeleop(m_drive, m_Vision,
    () -> {return -m_driverController.getLeftY();},
    () -> {return -m_driverController.getLeftX();},
    () -> {return -m_driverController.getRightX();},
    () -> {return SwerveConstants.isFieldRelative;},
    () -> {return SwerveConstants.isOpenLoop;}));
    // m_wrist.setDefaultCommand(new command_ManualWrist(m_wrist, null));
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
    m_driverController.a().onTrue(m_drive.toggleParkCommand());
    m_driverController.b().onTrue(m_LED.toggleAmpCoopCommand());
    m_driverController.leftStick().onTrue(m_drive.zeroGyroInstantCommand());
    m_driverController.rightStick().onTrue(m_drive.toggleThrottleInstantCommand());
    m_driverController.leftTrigger().and(m_driverController.rightTrigger().negate()).onTrue(
                                    Commands.parallel(
                                    // new command_AutoSpeakerWheels(m_wheels, m_drive), 
                                    m_drive.setDriveStateCommand(DRIVE_STATE.SHOOTER_PREP), 
                                    new command_AutoSpeakerWrist(m_wrist, m_drive)));
    m_driverController.leftTrigger().and(m_driverController.rightTrigger()).onTrue(
                                      new command_WaitUntilReady(/*m_wheels, */ m_wrist, m_drive)
    /* xd */                        /* .andThen(m_wheels.runFeederCommand().withTimeout(2))*/);
    // m_driverController.leftTrigger().negate().and(m_driverController.rightTrigger()).onTrue( //lmao
    //                                               m_wheels.runFeederCommand().withTimeout(2));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
  // }
}
