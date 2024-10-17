// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.robotType;
import static frc.robot.FieldConstants.isInSpeakerWing;
import static frc.robot.commands.IndexerCommands.stopIndexer;
import static frc.robot.commands.IntakeCommands.intakeDeployAndIntake;
import static frc.robot.commands.IntakeCommands.stopRollers;
import static frc.robot.commands.IntakeCommands.stowCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AutoAlignCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.rollers.Rollers;
import frc.robot.subsystems.intake.rollers.RollersIO;
import frc.robot.subsystems.intake.rollers.RollersIOSim;
import frc.robot.subsystems.intake.rollers.RollersIOTalonFX;
import frc.robot.subsystems.intake.wrist.IntakeWrist;
import frc.robot.subsystems.intake.wrist.IntakeWristIO;
import frc.robot.subsystems.intake.wrist.IntakeWristIOSim;
import frc.robot.subsystems.intake.wrist.IntakeWristIOTalonFX;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.wrist.ShooterWrist;
import frc.robot.subsystems.shooter.wrist.ShooterWristIO;
import frc.robot.subsystems.shooter.wrist.ShooterWristIOSim;
import frc.robot.subsystems.shooter.wrist.ShooterWristIOTalonFX;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Lookup;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
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
  private Drive drive;
  private ShooterWrist shooterWrist;
  private Flywheel flywheels;
  private Rollers rollers;
  private IntakeWrist intakeWrist;
  private Indexer indexer;

  /* Controllers */
  private final CommandXboxController driver =
      new CommandXboxController(ControllerConstants.xboxDriveID);
  private final CommandXboxController operator =
      new CommandXboxController(ControllerConstants.xboxOperatorID);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private final LoggedDashboardChooser<Command> m_Chooser;

  public RobotContainer() {
    switch (robotType) {
      case REALBOT -> {
        shooterWrist = new ShooterWrist(new ShooterWristIOTalonFX());
        flywheels = new Flywheel(new FlywheelIOTalonFX());
        rollers = new Rollers(new RollersIOTalonFX());
        intakeWrist = new IntakeWrist(new IntakeWristIOTalonFX());
        indexer = new Indexer(new IndexerIOTalonFX());
        drive =
            new Drive(
                new GyroIOPigeon2(true), Drive.createTalonFXModules(), Drive.createRealCameras());
      }

      case SIMBOT -> {
        shooterWrist = new ShooterWrist(new ShooterWristIOSim());
        flywheels = new Flywheel(new FlywheelIOSim());
        rollers = new Rollers(new RollersIOSim());
        intakeWrist = new IntakeWrist(new IntakeWristIOSim());
        indexer = new Indexer(new IndexerIOSim());
        drive = new Drive(new GyroIO() {}, Drive.createSimModules(), Drive.createSimCameras());
      }

        // Replayed robot, disable IO implementations
      case REPLAYBOT -> {
        shooterWrist = new ShooterWrist(new ShooterWristIO() {});
        flywheels = new Flywheel(new FlywheelIO() {});
        rollers = new Rollers(new RollersIO() {});
        intakeWrist = new IntakeWrist(new IntakeWristIO() {});
        indexer = new Indexer(new IndexerIO() {});
        drive = new Drive(new GyroIO() {}, Drive.createSimModules(), Drive.createSimCameras());
      }
      default -> {
        break;
      }
    }

    // NamedCommands.registerCommand("AutoAlignShoot", AutoAlignShootAnywhereCommand());
    NamedCommands.registerCommand("shoot bud", autoShoot());
    NamedCommands.registerCommand("stop bud", stopAll());

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
        drive.runVoltageTeleopFieldRelative(
            () ->
                new ChassisSpeeds(
                    -joystickDeadbandApply(driver.getLeftY()) * DriveConstants.MAX_LINEAR_SPEED,
                    -joystickDeadbandApply(driver.getLeftX()) * DriveConstants.MAX_LINEAR_SPEED,
                    -joystickDeadbandApply(driver.getRightX())
                        * DriveConstants.MAX_ANGULAR_SPEED)));

    driver.rightTrigger().whileTrue(deployAndIntake());

    driver
        .rightTrigger()
        .onFalse(
            Commands.parallel(
                stopIndexer(indexer), stopRollers(rollers), stowCommand(intakeWrist)));

    driver.y().onTrue(drive.zeroGyroCommand());
    /*  ---------------------
      Operator Controls
    ---------------------  */

    operator.a().whileTrue(indexer.setSpeed(0.4));
    operator.b().whileTrue(indexer.setSpeed(-0.4));
    operator.x().whileTrue(rollers.setSpeed(0.4));
    operator.y().whileTrue(rollers.setSpeed(-0.4));

    operator.rightTrigger().whileTrue(shoot());
    // operator.leftBumper().whileTrue(feedShot());

    // auto Stow
    // operator.leftTrigger().negate().and(operator.rightTrigger().negate()).onTrue(stowShooter());

    // stop and stow all
    operator.start().onTrue(stopAll());
  }

  public Command AutoAlignShootAnywhereCommand() {
    return Commands.parallel(
        AutoAlign(),
        rumbleIfNotSpeakerWing(),
        Commands.parallel(wristToSpeakerForever(), shoot()).onlyIf(() -> isInSpeakerWing(drive)));
  }

  public Command AutoAlign() {
    return Commands.sequence(
        Commands.waitUntil(() -> isInSpeakerWing(drive)),
        AutoAlignCommands.autoAlignSpeakerCommand(drive, driver)
            .onlyIf(() -> isInSpeakerWing(drive)));
  }

  public Command rumbleIfNotSpeakerWing() {
    return rumbleControllers().onlyIf(() -> !isInSpeakerWing(drive));
  }

  public Command shoot() {
    return Commands.sequence(
        indexer.prepNote(),
        flywheels.runFlywheelVelocity(45, 45),
        Commands.waitUntil(() -> flywheels.flywheelsSpunUp()),
        Commands.startEnd(
                () -> operator.getHID().setRumble(RumbleType.kBothRumble, 1),
                () -> operator.getHID().setRumble(RumbleType.kBothRumble, 0))
            .withTimeout(0.5),
        feedShot());
  }

  public Command autoShoot() {
    return Commands.parallel(shoot(), autoFeedShot());
  }

  // public Command shooting() {
  //   return Commands.parallel(
  //       flywheels.runFlywheelVelocity(45, 45),
  //       Commands.sequence(
  //           new WaitCommand(1), indexer.setSpeed(-0.4), new WaitCommand(1), indexer.setSpeed(0),
  // stopFlywheels()));
  // }

  public Command autoFeedShot() {
    return Commands.sequence(new WaitCommand(3), indexer.setSpeed(-0.4));
  }

  public Command stopAll() {
    return Commands.parallel(
        stowCommand(intakeWrist),
        rollers.setSpeed(0),
        stopIndexer(indexer),
        stowShooter(),
        stopFlywheels());
  }

  public Command feedShot() {
    return Commands.sequence(
        indexer.setSpeed(-0.4), new WaitCommand(0.25), stopFlywheels(), stopIndexer(indexer));
  }

  public Command stopFlywheels() {
    return flywheels.runVoltage(0);
  }

  public Command deployAndIntake() {
    return Commands.parallel(intakeDeployAndIntake(intakeWrist, rollers, indexer, driver));
  }

  public Command rumbleControllers() {
    return new StartEndCommand(
            () -> driver.getHID().setRumble(RumbleType.kBothRumble, 1),
            () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0))
        .alongWith(
            new StartEndCommand(
                () -> operator.getHID().setRumble(RumbleType.kBothRumble, 1),
                () -> operator.getHID().setRumble(RumbleType.kBothRumble, 0)));
  }

  public Command stowShooter() {
    return shooterWrist.PIDCommand(0);
  }

  public Command wristToSpeakerForever() {
    return shooterWrist.PIDCommandForever(this::getAngle);
  }

  private Transform2d getEstimatedTransform() {
    return new Transform2d(
        new Translation2d(
            drive.getVelocity().vxMetersPerSecond * 0.02,
            drive.getVelocity().vyMetersPerSecond * 0.02),
        new Rotation2d(0.0));
  }

  // Returns the estimated robot position

  private Pose2d getEstimatedPosition() {
    return drive.getPose().plus(getEstimatedTransform().inverse());
  }

  // Returns the distance between the robot's next estimated position and the
  // speaker position
  @AutoLogOutput(key = "DistanceAway")
  private double getEstimatedDistance() {
    Transform2d targetTransform =
        getEstimatedPosition()
            .minus(
                AllianceFlipUtil.apply(
                    new Pose2d(
                        -0.2,
                        AutoAlignCommands.autoAlignSpeakerPoseSetter(drive),
                        new Rotation2d(0))));
    return targetTransform.getTranslation().getNorm();
  }

  @AutoLogOutput(key = "PointedAtSpeaker")
  public boolean isPointedAtSpeaker() {
    return AutoAlignCommands.pointedAtSpeaker(drive);
  }

  @AutoLogOutput(key = "FlywheelsSpunUp")
  public boolean areSpunUp() {
    return flywheels.flywheelsSpunUp();
  }

  @AutoLogOutput(key = "AimedAtSpeaker")
  public boolean isAimedAtSpeaker() {
    return AutoAlignCommands.pointedAtSpeaker(drive) && shooterWrist.atSetpoint();
  }

  // Gets angle based on distance from speaker, taking into account the actual
  // shooting position
  @AutoLogOutput(key = "ShootAnywhereAngle")
  private double getAngle() {
    double angle = Lookup.getWristAngle(getEstimatedDistance());
    Logger.recordOutput("ShootAnywhereAngle", angle);
    return angle;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_Chooser.get();
  }

  private static double joystickDeadbandApply(double x) {
    // return MathUtil.applyDeadband(Math.abs(Math.pow(x, 3) * Math.signum(x)), 0.02);
    return MathUtil.applyDeadband(
        (Math.signum(x) * (1.01 * Math.pow(x, 2) - 0.0202 * x + 0.0101)), 0.02);
  }
}
