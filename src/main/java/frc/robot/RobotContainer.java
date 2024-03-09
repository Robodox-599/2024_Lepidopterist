// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.LEDConstants.LEDAnimation;
import frc.robot.commands.Autos;
import frc.robot.commands.cGroup_Shooter;
import frc.robot.commands.command_AutoSpeaker;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.command_DriveTeleop;
import frc.robot.commands.command_ToWristAndSpeed;
import frc.robot.subsystems.subsystem_DriveTrain;
import frc.robot.subsystems.subsystem_Indexer;
import frc.robot.subsystems.subsystem_Intake;
import frc.robot.subsystems.subsystem_LED;
import frc.robot.subsystems.subsystem_Shooter;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final subsystem_Shooter m_Shooter = new subsystem_Shooter();
  private final subsystem_Indexer m_Indexer = new subsystem_Indexer();
  private final subsystem_Intake m_Intake = new subsystem_Intake();
  private final subsystem_LED m_LED = new subsystem_LED();

  /* Controllers */  
  private final static CommandXboxController driver = new CommandXboxController(ControllerConstants.xboxDriveID);
  private final CommandXboxController operator = new CommandXboxController(ControllerConstants.xboxOperatorID);

  /* Sendable Chooser */
  private final SendableChooser<Command> m_Chooser = AutoBuilder.buildAutoChooser();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    // NamedCommands.registerCommand(null, getAutonomousCommand());
    m_Chooser.addOption("Taxi", Autos.runAutoPath(m_DriveTrain, "Taxi"));
    m_Chooser.addOption("2 Note Left", Autos.runAutoPath(m_DriveTrain, "2N_L_C"));
    m_Chooser.addOption("2 Note Mid", Autos.runAutoPath(m_DriveTrain, "2N_M_C"));
    m_Chooser.addOption("3 Note Right", Autos.runAutoPath(m_DriveTrain, "3N_R_C"));
    m_Chooser.addOption("3 Note Left", Autos.runAutoPath(m_DriveTrain, "3N_L_C"));
    m_Chooser.addOption("4 Note", Autos.runAutoPath(m_DriveTrain, "4N_C"));

    m_DriveTrain.setDefaultCommand(new command_DriveTeleop(m_DriveTrain, 
                                                          () -> {return -driver.getLeftY();},
                                                          () -> {return -driver.getLeftX();},
                                                          () -> {return -driver.getRightX();},
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
    driver.y().onTrue(m_DriveTrain.zeroGyroInstantCommand());
    driver.leftStick().onTrue(m_DriveTrain.toggleLinearThrottleCommand());
    driver.rightStick().onTrue(m_DriveTrain.toggleAngularThrottleCommand());
    //arm speaker
    driver.leftTrigger().and(driver.rightTrigger().negate()).whileTrue(
                                    Commands.parallel(
                                    new command_AutoSpeaker(m_DriveTrain, m_Shooter)));
    //shoot when ready
    driver.leftTrigger().and(driver.rightTrigger()).whileTrue(
                                      m_Shooter.waitUntilReady(m_DriveTrain.getPose())
    /* xd */                         .andThen(Commands.parallel(
      m_LED.setAnimationCommand(LEDAnimation.Shooting, () -> {return 0.5;}), 
      m_Shooter.feedersStartEnd().withTimeout(2))));

    //manual shoot
    driver.leftTrigger().negate().and(driver.rightTrigger()).whileTrue( //lmao
                                                  m_Shooter.feedersStartEnd().withTimeout(2));

    //intake command
    driver.leftBumper().whileTrue(Commands.parallel(
      runIntakeSequence(m_Intake, m_Indexer)
    )); //TODO: Add Retraction upon release

    //amp command
    driver.rightBumper().whileTrue(cGroup_Shooter.scoreAmp(m_Shooter, m_LED));

    //stowall
    driver.y().onTrue(
      Commands.parallel(
        m_Intake.stowCommand(), 
        m_Indexer.stopIndexerCommand(),
        new command_ToWristAndSpeed(m_Shooter, () -> ShooterConstants.minShootAngle,
        () -> ShooterConstants.StowSpeed)));

    //yay operator!!!
    operator.a().onTrue(m_LED.toggleAmpCoopCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_Chooser.getSelected();
  }

  public static Command runIntakeSequence(subsystem_Intake intake, subsystem_Indexer indexer){
    return Commands.sequence(
      Commands.parallel(
        Commands.sequence(
        intake.moveWristCommand(() -> {return IntakeConstants.kWristExtendVal;}), 
        intake.runIntakeuntilBeamBreak()), 
        indexer.runIndexerCommand()), 
      Commands.parallel( //bleugh
        rumbleControllerStartEnd().withTimeout(2.0), 
        intake.moveWristCommand(() -> {return IntakeConstants.kWristRetractVal;}), 
        Commands.sequence(indexer.runIndexerUntilBeamBreak(), 
          indexer.runIndexerStartEnd().withTimeout(0.5))));
  }

  public static Command rumbleControllerStartEnd(){
    //wtf
    return new StartEndCommand(() -> driver.getHID().setRumble(RumbleType.kBothRumble, 1), () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0), new Subsystem[0]);
  }
}
