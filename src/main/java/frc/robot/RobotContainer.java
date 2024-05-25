// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.FlywheelSetpoints;
import frc.robot.Constants.ShooterConstants.WristSepoints;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.command_ManualShooter;
import frc.robot.commands.command_ToWristAndSpeed;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.subsystem_Indexer;
import frc.robot.subsystems.intake.subsystem_Intake;
import frc.robot.subsystems.led.subsystem_LED;
import frc.robot.subsystems.shooter.subsystem_Shooter;
import frc.robot.subsystems.vision.subsystem_Vision;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  /* Subsystems */
  public Drive drive;
  //   private final VisionSubsystem vision = new VisionSubsystem)(;
  private final subsystem_Vision m_Vision = new subsystem_Vision();
  private final subsystem_Shooter m_Shooter = new subsystem_Shooter();
  private final subsystem_Indexer m_Indexer = new subsystem_Indexer();
  private final subsystem_Intake m_Intake = new subsystem_Intake();
  private final subsystem_LED m_LED = new subsystem_LED();

  /* Controllers */
  private final CommandXboxController driver =
      new CommandXboxController(ControllerConstants.xboxDriveID);
  private final CommandXboxController operator =
      new CommandXboxController(ControllerConstants.xboxOperatorID);

  /* Sendable Chooser */
  // private final LoggedDashboardChooser<Command> m_Chooser;
  // private final SendableChooser<Command> m_Chooser = new SendableChooser<>();
  // new LoggedDashboardChooser<>("Auto Choices");
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private final LoggedDashboardChooser<Command> m_Chooser;

  public RobotContainer() {

    // vision = new VisionSubsystem(new VisionIO[] {});
    CameraServer.startAutomaticCapture(0);
    // m_Chooser = new LoggedDashboardChooser<>("Auto Choices");

    switch (Constants.getRobot()) {
      case REALBOT -> {
        drive =
            new Drive(
                new GyroIOPigeon2(true),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3),
                Drive.createRealCameras());
      }

      case SIMBOT -> {
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                Drive.createSimCameras());
      }

        // Replayed robot, disable IO implementations
      case REPLAYBOT -> {
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                Drive.createSimCameras());
      }
      default -> {
        break;
      }
    }
    // Configure the trigger bindings
    // m_DriveTrain.setDefaultCommand(
    //     new command_DriveTeleop(
    //         m_DriveTrain,
    //         m_Vision,
    //         () -> -driver.getLeftY(),
    //         () -> -driver.getLeftX(),
    //         () -> -driver.getRightX(),
    //         () -> SwerveConstants.isFieldRelative,
    //         () -> SwerveConstants.isOpenLoop));

    // m_Intake.setDefaultCommand(new command_ManualIntakeWrist(m_Intake, () ->
    // -operator.getRightY()));

    m_Shooter.setDefaultCommand(new command_ManualShooter(m_Shooter, () -> -operator.getLeftY()));

    // LOGGED DASHBOARD NOW CALLS THE AUTOS IN AUTOBUILDER. THAT THEN LOOKS FOR THE AUTOS AND SENDS
    // THEM TO THE LOGGED DASHBOARD. :)

    NamedCommands.registerCommand(
        "Shoot Subwoofer",
        Commands.sequence(
            // Autos.runAutoPath(drive, "Taxi Auto"),
            m_Shooter.initialSetWristAngle(),
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
            new command_ToWristAndSpeed(
                m_Shooter,
                () -> ShooterConstants.WristSepoints.testSpeakerWrist,
                () -> ShooterConstants.FlywheelSetpoints.SpeakerSpeed),
            new WaitUntilCommand(m_Shooter.isReadyToShoot()),
            Commands.parallel(
                m_Shooter.forwardsFeederStartEnd().withTimeout(2.0),
                m_Indexer.runIndexerShootStartEnd().withTimeout(2.0)),
            m_Shooter.stowShooter()) // TODO: Set timeout to 0.75 after indexer re-attached
        );
    m_Chooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    configureBindings();
  }

  // private final SendableChooser<Command> m_Chooser = AutoBuilder.buildAutoChooser();
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
    /*  -------------------
      Driver Controls
    -------------------  */
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
    // driver.a().onTrue(m_DriveTrain.parkCommand());
    // driver.x().onTrue(m_DriveTrain.invertGyroInstantCommand());
    // driver.y().onTrue(m_DriveTrain.toggleGyroCommand());

    // driver.leftStick().onTrue(m_DriveTrain.toggleLinearThrottleCommand());
    // driver.rightStick().onTrue(m_DriveTrain.toggleAngularThrottleCommand());

    // // DPAD
    // driver.povUp().onTrue(m_DriveTrain.DPADCommand(DPAD.UP));
    // driver.povUpRight().onTrue(m_DriveTrain.DPADCommand(DPAD.UPRIGHT));
    // driver.povRight().onTrue(m_DriveTrain.DPADCommand(DPAD.RIGHT));
    // driver.povDownRight().onTrue(m_DriveTrain.DPADCommand(DPAD.DOWNRIGHT));
    // driver.povDown().onTrue(m_DriveTrain.DPADCommand(DPAD.DOWN));
    // driver.povDownLeft().onTrue(m_DriveTrain.DPADCommand(DPAD.DOWNLEFT));
    // driver.povLeft().onTrue(m_DriveTrain.DPADCommand(DPAD.LEFT));
    // driver.povUpLeft().onTrue(m_DriveTrain.DPADCommand(DPAD.UP));

    // Intake Command
    driver
        .leftBumper()
        .whileTrue(
            Commands.parallel(
                m_Intake.autoSpeakerIntake().andThen(rumbleControllers().withTimeout(1)),
                m_Indexer.runIndexerUntilBeamBreak()));
    driver.leftBumper().onFalse(m_Intake.stowCommand().alongWith(m_Indexer.stopIndexerCommand()));

    // intake to amp command
    driver
        .leftTrigger()
        .whileTrue(
            Commands.parallel(
                m_Intake.autoAmpIntake().andThen(rumbleControllers().withTimeout(1)),
                m_Indexer.runIndexerUntilBeamBreak()));
    driver.leftTrigger().onFalse(m_Intake.stowCommand().alongWith(m_Indexer.stopIndexerCommand()));

    // intake source
    driver
        .rightBumper()
        .whileTrue(
            Commands.sequence(
                m_Shooter.autoSourceShooter().alongWith(m_Indexer.sourceIndexer()),
                new InstantCommand(() -> m_Shooter.stopFeeder(), m_Shooter)));
    driver
        .rightBumper()
        .onFalse(
            m_Shooter
                .stowAndStopShooter()
                .andThen(m_Indexer.sourceIndexer())
                .withTimeout(0.5)
                .andThen(m_Shooter.stowShooter()));

    // stow all
    driver
        .start()
        .onTrue(
            Commands.parallel(
                m_Intake.stowCommand(), m_Indexer.stopIndexerCommand(), m_Shooter.stowShooter()));

    /*  ---------------------
      Operator Controls
    ---------------------  */

    operator.a().whileTrue(m_Indexer.runIndexerStartEnd());
    operator.b().whileTrue(m_Indexer.runIndexerBackwardsStartEnd());
    operator.x().whileTrue(m_Intake.runIntakeFwdCMD());
    operator.y().whileTrue(m_Intake.runIntakeBackCMD());

    // operator.povLeft().onTrue(m_DriveTrain.resetSubwoofer(() -> 0.0));
    // operator.povUp().onTrue(m_DriveTrain.resetSubwoofer(() -> 1.0));
    // operator.povRight().onTrue(m_DriveTrain.resetSubwoofer(() -> 2.0));
    // operator.povLeft().onTrue(m_DriveTrain.resetSubwoofer(subwooferSide.LEFT));
    // operator.povUp().onTrue(m_DriveTrain.resetSubwoofer(subwooferSide.CENTER);
    // operator.povRight().onTrue(m_DriveTrain.resetSubwoofer(subwooferSide.RIGHT);

    // arm speaker
    // operator.leftTrigger().and(operator.rightTrigger().negate()).whileTrue(
    //                                               new command_AutoSpeaker(m_DriveTrain,
    // m_Shooter, () -> false));

    // //shoot when ready
    // operator.leftTrigger().and(operator.rightTrigger()).onTrue(
    //                                               m_Shooter.waitUntilReady().andThen(
    //                                                 shootSequence(m_Indexer,
    // m_Shooter).withTimeout(1.0)));

    operator
        .leftTrigger()
        .and(operator.rightTrigger().negate())
        .whileTrue(scoreAmp().withTimeout(1.0));
    operator.leftTrigger().and(operator.rightTrigger().negate()).onFalse(m_Shooter.stowShooter());

    // manual shoot
    operator
        .leftTrigger()
        .negate()
        .and(operator.rightTrigger())
        .whileTrue(shootSequence(m_Indexer, m_Shooter));
    // driver.leftTrigger().negate().and(driver.rightTrigger()).whileTrue(shootSequence(m_Indexer,
    // m_Shooter));

    // auto Stow
    operator
        .leftTrigger()
        .negate()
        .and(operator.rightTrigger().negate())
        .onTrue(m_Shooter.stowShooter());

    // Amp Score
    // operator.leftBumper().whileTrue(m_Intake.ampScoreCommand());
    // operator.leftBumper().onFalse(m_Intake.stowCommand());
    // operator.povDownRight().whileTrue(m_Intake.ampFling());
    // operator.povDown().onTrue(m_DriveTrain.flipBotCommand());

    // subwoofer setpoint
    operator
        .leftBumper()
        .and(operator.rightBumper().negate())
        .whileTrue(
            new command_ToWristAndSpeed(
                m_Shooter,
                () -> ShooterConstants.WristSepoints.testSpeakerWrist,
                () -> ShooterConstants.FlywheelSetpoints.SpeakerSpeed));
    operator
        .leftBumper()
        .and(operator.rightBumper())
        .whileTrue(
            m_Shooter
                .waitUntilReady()
                .andThen(shootSequence(m_Indexer, m_Shooter).withTimeout(1.0)));
    operator
        .leftBumper()
        .negate()
        .and(operator.rightBumper().negate())
        .onTrue(m_Shooter.stowShooter());

    // Stow All
    operator
        .start()
        .onTrue(
            Commands.parallel(
                m_Intake.stowCommand(), m_Indexer.stopIndexerCommand(), m_Shooter.stowShooter()));

    operator
        .back()
        .whileTrue(m_Shooter.forwardsFeederStartEnd().alongWith(m_Indexer.runIndexerStartEnd()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_Chooser.get();
  }

  public Command shootSequence(subsystem_Indexer indexer, subsystem_Shooter shooter) {
    return m_Shooter
        .setFlywheelSpeedCommand(() -> FlywheelSetpoints.testFlywheelSetpoint)
        .andThen(m_Shooter.waitUntilReady())
        .andThen(
            Commands.parallel(
                m_Indexer.runIndexerShootStartEnd(),
                m_Shooter.forwardsFeederStartEnd(),
                rumbleControllers()));
  }

  public Command ampShootSequence(subsystem_Indexer indexer, subsystem_Shooter shooter) {
    return Commands.parallel(
        m_Indexer.runIndexerShootStartEnd(),
        m_Shooter
            .forwardsFeederStartEnd()
            .withTimeout(0.1)
            .andThen(
                new InstantCommand(() -> m_Shooter.stopFeeder(), m_Shooter)
                    .andThen(Commands.waitSeconds(0.1))
                    .andThen(m_Shooter.forwardsFeederStartEnd().withTimeout(2.5))),
        rumbleControllers());
  }

  public Command rumbleControllers() {
    // wtf
    return new StartEndCommand(
            () -> driver.getHID().setRumble(RumbleType.kBothRumble, 1),
            () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0))
        .alongWith(
            new StartEndCommand(
                () -> operator.getHID().setRumble(RumbleType.kBothRumble, 1),
                () -> operator.getHID().setRumble(RumbleType.kBothRumble, 0)));
  }

  public Command scoreAmp() {
    return Commands.sequence(
        new InstantCommand(() -> m_Shooter.setDesiredShootAngle(WristSepoints.ampWrist), m_Shooter),
        m_Shooter.setFlywheelSpeedCommand(() -> FlywheelSetpoints.AmpSpeed),
        m_Shooter.waitUntilReady(),
        ampShootSequence(m_Indexer, m_Shooter));
  }
}
