// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.LEDConstants.LEDAnimation;
import frc.robot.Constants.ShooterConstants.WristSepoints;
import frc.robot.Constants.SwerveConstants.DPAD;
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
import frc.robot.subsystems.subsystem_Vision;
import frc.robot.subsystems.subsystem_Indexer;
import frc.robot.subsystems.subsystem_Intake;
import frc.robot.subsystems.subsystem_LED;
import frc.robot.subsystems.subsystem_Shooter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
    // Configure the trigger bindings
    m_DriveTrain.setDefaultCommand(new command_DriveTeleop(m_DriveTrain, m_Vision,
                                                          () -> -driver.getLeftY(),
                                                          () -> -driver.getLeftX(),
                                                          () -> -driver.getRightX(),
                                                          () -> SwerveConstants.isFieldRelative,
                                                          () -> SwerveConstants.isOpenLoop));

    // m_Shooter.setDefaultCommand(new command_ToWristAndSpeed(m_Shooter, 
    //                             () -> ShooterConstants.minNetAngle, 
    //                             () -> ShooterConstants.AmpSpeed));
    m_Shooter.setDefaultCommand(m_Shooter.manualWrist(() -> operator.getLeftY(), () -> operator.b().getAsBoolean()));

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

    driver.y().onTrue(m_DriveTrain.zeroGyroInstantCommand());
    driver.leftStick().onTrue(m_DriveTrain.toggleLinearThrottleCommand());
    driver.rightStick().onTrue(m_DriveTrain.toggleAngularThrottleCommand());

    //arm speaker
    driver.leftTrigger().and(driver.rightTrigger().negate()).whileTrue(
                                                  new command_AutoSpeaker(m_DriveTrain, m_Shooter));
    
    //shoot when ready
    driver.leftTrigger().and(driver.rightTrigger()).whileTrue(
                                      m_Shooter.waitUntilReady(m_DriveTrain.getPose())
                                    .andThen(Commands.parallel(
                                    m_Indexer.runIndexerStartEnd().withTimeout(1), 
                                    m_Shooter.forwardsFeederStartEnd().withTimeout(1), 
                                    m_LED.setAnimationCommand(LEDAnimation.Shooting, () -> 0.5))));
    
    driver.leftTrigger().negate().and(driver.rightTrigger()).whileTrue(m_Shooter.forwardsFeederStartEnd());                      

    driver.leftTrigger().onFalse(new command_ToWristAndSpeed(m_Shooter, 
                                () -> ShooterConstants.WristSepoints.minShootAngle, 
                                () -> ShooterConstants.FlywheelSetpoints.AmpSpeed));

    // Intake Command
    driver.leftBumper().whileTrue(runIntakeSequence(m_Intake, m_Indexer));
    driver.leftBumper().onFalse(Commands.parallel(m_Intake.stowCommand(), m_Indexer.stopIndexerCommand()));

    // Amp Command
    driver.rightBumper().whileTrue(cGroup_Shooter.scoreAmp(m_Shooter, m_LED));
    driver.rightBumper().onFalse(new command_ToWristAndSpeed(m_Shooter, 
                                () -> ShooterConstants.WristSepoints.minShootAngle, 
                                () -> ShooterConstants.FlywheelSetpoints.AmpSpeed));

    // Stow All
    driver.x().onTrue(
      Commands.parallel(
        m_Intake.stowCommand(), 
        m_Indexer.stopIndexerCommand(),
        new command_ToWristAndSpeed(m_Shooter, () -> WristSepoints.minShootAngle,
        () -> ShooterConstants.FlywheelSetpoints.AmpSpeed)));

    //stow js wrist
    operator.y().onTrue(
      new command_ToWristAndSpeed(m_Shooter, 
                                () -> ShooterConstants.WristSepoints.minShootAngle, 
                                () -> ShooterConstants.FlywheelSetpoints.AmpSpeed));
    

    operator.x().onTrue(Commands.parallel(m_Indexer.runIndexerUntilBeamBreak(),
                                          m_Intake.runIntakeuntilBeamBreak()));

    operator.a().onTrue(m_LED.toggleAmpCoopCommand());
    operator.b().whileTrue(m_Indexer.runIndexerStartEnd());

    operator.start().whileTrue(m_Intake.runIntakeBackCMD());
    // Manual Wrist (moved to defualt command)
    // operator.axisGreaterThan(XboxController.Axis.kLeftY.value, ControllerConstants.deadband).or(
    //           operator.axisLessThan(XboxController.Axis.kLeftY.value, -ControllerConstants.deadband)).whileTrue(
    //             m_Shooter.manualWrist(() -> operator.getLeftY(), ()-> operator.b().getAsBoolean())); // TODO: Add max speed

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
    m_Chooser.addOption("Taxi", Autos.runAutoPath(m_DriveTrain, "Taxi"));
    m_Chooser.addOption("2 Note Left", Autos.runAutoPath(m_DriveTrain, "2 Note Left"));
    m_Chooser.addOption("2 Note Mid", Autos.runAutoPath(m_DriveTrain, "2 Note Mid"));
    m_Chooser.addOption("3 Note Right", Autos.runAutoPath(m_DriveTrain, "3 Note Right"));
    m_Chooser.addOption("3 Note Left", Autos.runAutoPath(m_DriveTrain, "3 Note Left"));
    m_Chooser.addOption("4 Note", Autos.runAutoPath(m_DriveTrain, "4 Note"));

    NamedCommands.registerCommand("Run Intake", runIntakeSequence(m_Intake, m_Indexer));

    NamedCommands.registerCommand("Preload Left", new command_ToWristAndSpeed(m_Shooter,
                                                      () -> AutoConstants.AutoWristSetpoints[0], 
                                                      () -> AutoConstants.AutoFlywheelSetpoints[0]));

    NamedCommands.registerCommand("First Row Left", new command_ToWristAndSpeed(m_Shooter,
                                                        () -> AutoConstants.AutoWristSetpoints[1], 
                                                        () -> AutoConstants.AutoFlywheelSetpoints[1]));

     NamedCommands.registerCommand("First Row Mid", new command_ToWristAndSpeed(m_Shooter,
                                                        () -> AutoConstants.AutoWristSetpoints[2], 
                                                        () -> AutoConstants.AutoFlywheelSetpoints[2]));

     NamedCommands.registerCommand("First Row Right", new command_ToWristAndSpeed(m_Shooter,
                                                          () -> AutoConstants.AutoWristSetpoints[3], 
                                                          () -> AutoConstants.AutoFlywheelSetpoints[3]));

     NamedCommands.registerCommand("Second Row (5 note)", new command_ToWristAndSpeed(m_Shooter,
                                                      () -> AutoConstants.AutoWristSetpoints[4], 
                                                      () -> AutoConstants.AutoFlywheelSetpoints[4]));

     NamedCommands.registerCommand("Second Row (complement)", new command_ToWristAndSpeed(m_Shooter,
                                                      () -> AutoConstants.AutoWristSetpoints[5], 
                                                      () -> AutoConstants.AutoFlywheelSetpoints[5]));

     NamedCommands.registerCommand("Preload Mid", new command_ToWristAndSpeed(m_Shooter,
                                                      () -> AutoConstants.AutoWristSetpoints[6], 
                                                      () -> AutoConstants.AutoFlywheelSetpoints[6]));

     NamedCommands.registerCommand("Preload Right", new command_ToWristAndSpeed(m_Shooter,
                                                        () -> AutoConstants.AutoWristSetpoints[7], 
                                                        () -> AutoConstants.AutoFlywheelSetpoints[7]));

    SmartDashboard.putData("Auto Chooser", m_Chooser);
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
    return Commands.parallel(
      intake.autoIntakeCommand().andThen(rumbleControllerStartEnd().withTimeout(1.0)),
      indexer.runIndexerUntilBeamBreak().andThen(indexer.runIndexerStartEnd().withTimeout(1))); 
  }

  public static Command rumbleControllerStartEnd(){
    //wtf
    return new StartEndCommand(
              () -> driver.getHID().setRumble(RumbleType.kBothRumble, 1),
              () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0), 
              new Subsystem[0]);
  }
}
