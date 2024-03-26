// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.LEDConstants.LEDAnimation;
import frc.robot.Constants.LEDConstants.LEDColor;
import frc.robot.Constants.ShooterConstants.WristSepoints;
import frc.robot.Constants.SwerveConstants.DPAD;
import frc.robot.commands.Autos;
import frc.robot.commands.command_AutoSpeaker;
// import frc.robot.commands.cGroup_Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.command_DriveTeleop;
import frc.robot.commands.command_ManualIntakeWrist;
import frc.robot.commands.command_ToWristAndSpeed;
import frc.robot.subsystems.subsystem_DriveTrain;
import frc.robot.subsystems.subsystem_Indexer;
import frc.robot.subsystems.subsystem_Vision;
import frc.robot.subsystems.subsystem_Indexer;
import frc.robot.subsystems.subsystem_Intake;
import frc.robot.subsystems.subsystem_LED;
import frc.robot.subsystems.subsystem_Shooter;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
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
  private final subsystem_Vision m_Vision = new subsystem_Vision();
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
    addAutos();
    // Configure the trigger bindings
    m_DriveTrain.setDefaultCommand(new command_DriveTeleop(m_DriveTrain, m_Vision,
                                                          () -> -driver.getLeftY(),
                                                          () -> -driver.getLeftX(),
                                                          () -> -driver.getRightX(),
                                                          () -> SwerveConstants.isFieldRelative,
                                                          () -> SwerveConstants.isOpenLoop));

    // m_Intake.setDefaultCommand(new command_ManualIntakeWrist(m_Intake, () -> -operator.getLeftY()));

    // m_Shooter.setDefaultCommand(m_Shooter.manualWrist(() -> -operator.getLeftY(), () -> operator.y().getAsBoolean()));
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

    driver.y().onTrue(m_DriveTrain.toggleGyrCommand());
    driver.x().onTrue(m_DriveTrain.invertGyroInstantCommand());
    driver.leftStick().onTrue(m_DriveTrain.toggleLinearThrottleCommand());
    driver.rightStick().onTrue(m_DriveTrain.toggleAngularThrottleCommand());

    //arm speaker
    driver.leftTrigger().and(driver.rightTrigger().negate()).whileTrue(
                                                  new command_AutoSpeaker(m_DriveTrain, m_Shooter));
    
    operator.leftTrigger().and(operator.rightTrigger().negate()).whileTrue(
                                                 new command_ToWristAndSpeed(m_Shooter, 
                                                  () -> ShooterConstants.WristSepoints.testSpeakerWrist,
                                                  // () -> ShooterConstants.FlywheelSetpoints.SpeakerSpeed));
                                                  () -> 0.0));
    // operator.rightTrigger().and(operator.leftTrigger().negate()).whileTrue(Commands.sequence(
    //                                               new command_ToWristAndSpeed(m_Shooter, 
    //                                               () -> ShooterConstants.WristSepoints.testSpeakerWrist,
    //                                               () -> ShooterConstants.FlywheelSetpoints.SpeakerSpeed),
    //                                               new WaitCommand(1), 
    //                                               Commands.parallel(
    //                                               rumbleControllerStartEnd().withTimeout(1),
    //                                               m_Indexer.runIndexerStartEnd().withTimeout(1), 
    //                                               m_Shooter.forwardsFeederStartEnd().withTimeout(1))));
    
    //shoot when ready
    operator.leftTrigger().and(operator.rightTrigger()).whileTrue(
                                    Commands.parallel(
                                    rumbleControllerStartEnd().withTimeout(1),
                                    m_Indexer.runIndexerStartEnd().withTimeout(1), 
                                    m_Shooter.forwardsFeederStartEnd().withTimeout(1)));
    
    operator.leftTrigger().negate().and(operator.rightTrigger()).whileTrue(m_Shooter.forwardsFeederStartEnd());                      

    // TODO: Slow Shooter wrist when going to stow
    operator.leftTrigger().negate().onTrue(m_Shooter.stowShooter());
    // operator.rightTrigger().onFalse(m_Shooter.stowShooter());

    // Intake Speaker Command
    driver.leftBumper().whileTrue(runIntakeSequence(m_Intake, m_Indexer, m_LED, () -> false, () -> false));
    driver.leftBumper().onFalse(Commands.parallel(m_Intake.stowCommand()));

    // intake amp command
    driver.rightBumper().whileTrue(runIntakeSequence(m_Intake, m_Indexer, m_LED, () -> true, () -> false));
    driver.rightBumper().onFalse(m_Intake.stowCommand());
    
    // Amp Score
    operator.leftBumper().whileTrue(m_Intake.ampScoreCommand());
    operator.leftBumper().onFalse(m_Intake.stowCommand());

    // Stow All
    operator.start().onTrue(
      Commands.parallel(
        m_Intake.stowCommand(), 
        m_Indexer.stopIndexerCommand(),
        m_Shooter.stowShooter()));

     driver.start().onTrue(
      Commands.parallel(
        m_Intake.stowCommand(), 
        m_Indexer.stopIndexerCommand(),
        m_Shooter.stowShooter()));

    //LEDs
    // operator.a().onTrue(m_LED.toggleAmpCoopCommand());
    driver.a().whileTrue(m_Intake.runIntakeFwdCMD());
    
    //manual indexer
    operator.a().whileTrue(m_Indexer.runIndexerBackwardsStartEnd());

    operator.b().whileTrue(m_Indexer.runIndexerStartEnd());

    // manual outtake
    operator.back().whileTrue(m_Intake.runIntakeBackCMD());

    //use for testing only discard when done. 
    // operator.leftTrigger().onTrue(new InstantCommand(() -> m_Intake.runIntake(false), m_Intake)); //run max speed 
                                                                                          //run at normal speed just intake. (use to get current)
        
    // run intake
    // operator.x().whileTrue(Commands.parallel(m_Indexer.runIndexerUntilBeamBreak().andThen(m_Indexer.runIndexerStartEnd().withTimeout(IndexerConstants.extraIndexerTime)),
    //                                         m_Intake.runIntakeuntilBeamBreak()));
    
    //wrist stow
    // operator.y().onTrue(
    //   new command_ToWristAndSpeed(m_Shooter, 
    //                             () -> ShooterConstants.WristSepoints.minShootAngle, 
    //                             () -> ShooterConstants.FlywheelSetpoints.StowSpeed));

    // operator.leftBumper().whileTrue(Commands.parallel(m_Intake.backwardsIntakeCommand(), 
    //                                                   m_Indexer.runIndexerBackwardsUntilBeamBreak().andThen(m_Indexer.runIndexerBackwardsStartEnd().
    //                                                       withTimeout(IndexerConstants.backwardsIndexerTime))));

    // operator.leftBumper().onFalse(Commands.parallel(m_Intake.stowCommand(), m_Indexer.stopIndexerCommand()));

    //DPAD
    driver.povUp().onTrue(m_DriveTrain.DPADCommand(DPAD.UP));
    driver.povUpRight().onTrue(m_DriveTrain.DPADCommand(DPAD.UPRIGHT));
    driver.povRight().onTrue(m_DriveTrain.DPADCommand(DPAD.RIGHT));
    driver.povDownRight().onTrue(m_DriveTrain.DPADCommand(DPAD.DOWNRIGHT));
    driver.povDown().onTrue(m_DriveTrain.DPADCommand(DPAD.DOWN));
    driver.povDownLeft().onTrue(m_DriveTrain.DPADCommand(DPAD.DOWNLEFT));
    driver.povLeft().onTrue(m_DriveTrain.DPADCommand(DPAD.LEFT));
    driver.povUpLeft().onTrue(m_DriveTrain.DPADCommand(DPAD.UP));
  }

  public void addAutos(){
    NamedCommands.registerCommand("Extend Intake", runIntakeSequence(m_Intake, m_Indexer, m_LED, () -> false, () -> true).withTimeout(2));
    NamedCommands.registerCommand("Stow Intake", m_Intake.stowCommand());
    NamedCommands.registerCommand("Shoot Subwoofer", new command_ToWristAndSpeed(m_Shooter, 
                                                  () -> ShooterConstants.WristSepoints.testSpeakerWrist,
                                                  () -> ShooterConstants.FlywheelSetpoints.SpeakerSpeed).until(m_Shooter.isReadyToShoot())
                                                .andThen(Commands.parallel(m_Shooter.forwardsFeederStartEnd(), 
                                                                          m_Indexer.runIndexerStartEnd())).withTimeout(0.75).andThen(m_Shooter.stowShooter()));
    NamedCommands.registerCommand("Shoot Line", new command_AutoSpeaker(m_DriveTrain, m_Shooter).until(m_Shooter.isReadyToShoot())
                                                .andThen(Commands.parallel(m_Shooter.forwardsFeederStartEnd(), 
                                                                          m_Indexer.runIndexerStartEnd())).withTimeout(0.75).andThen(m_Shooter.stowShooter()));
    NamedCommands.registerCommand(null, getAutonomousCommand());
    m_Chooser.addOption("Taxi Auto", Autos.runAutoPath(m_DriveTrain, "Taxi Auto"));
    m_Chooser.addOption("Taxi Auto Left", Autos.runAutoPath(m_DriveTrain, "Taxi Auto Left"));
    m_Chooser.addOption("Do Nothing Auto", Autos.runAutoPath(m_DriveTrain, "Do Nothing Auto"));
    m_Chooser.addOption("2 Note Left", Autos.runAutoPath(m_DriveTrain, "2 Note Left"));
    m_Chooser.addOption("2 Note Mid", Autos.runAutoPath(m_DriveTrain, "2 Note Mid"));
    m_Chooser.addOption("2 Note Right", Autos.runAutoPath(m_DriveTrain, "2 Note Right"));
    m_Chooser.addOption("RIPA", Autos.runAutoPath(m_DriveTrain, "RIPA"));
    SmartDashboard.putData("Auto Chooser", m_Chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_Chooser.getSelected();
    // return Autos.runAutoPath(m_DriveTrain, "Taxi Auto");
  }

  public static Command runIntakeSequence(subsystem_Intake intake, subsystem_Indexer indexer, subsystem_LED led, BooleanSupplier isAmp, BooleanSupplier isAuto){
    return Commands.parallel(
      intake.autoIntakeCommand(isAmp, isAuto).andThen(rumbleControllerStartEnd().withTimeout(ControllerConstants.rumbleTime)),
      indexer.runIndexerUntilBeamBreak().andThen(indexer.runIndexerStartEnd().withTimeout(IndexerConstants.extraIndexerTime))
      ); 
  }
  

  public static Command rumbleControllerStartEnd(){
    //wtf
    return new StartEndCommand(
              () -> driver.getHID().setRumble(RumbleType.kBothRumble, 1),
              () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0));
  }
}