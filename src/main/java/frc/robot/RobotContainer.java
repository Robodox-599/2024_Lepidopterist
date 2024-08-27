// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.robotType;
import static frc.robot.FieldConstants.*;
import static frc.robot.commands.IndexerCommands.*;
import static frc.robot.commands.IntakeCommands.*;
import static frc.robot.subsystems.indexer.IndexerConstants.*;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.bottomFlywheelVelocityRPM;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.topFlywheelVelocityRPM;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AutoAlignCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
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
                new GyroIOPigeon2(true),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3),
                Drive.createRealCameras());
      }

      case SIMBOT -> {
        shooterWrist = new ShooterWrist(new ShooterWristIOSim());
        flywheels = new Flywheel(new FlywheelIOSim());
        rollers = new Rollers(new RollersIOSim());
        intakeWrist = new IntakeWrist(new IntakeWristIOSim());
        indexer = new Indexer(new IndexerIOSim());
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
        shooterWrist = new ShooterWrist(new ShooterWristIO() {});
        flywheels = new Flywheel(new FlywheelIO() {});
        rollers = new Rollers(new RollersIO() {});
        intakeWrist = new IntakeWrist(new IntakeWristIO() {});
        indexer = new Indexer(new IndexerIO() {});
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

    // NamedCommands.registerCommand("AutoAlignShoot", AutoAlignShootAnywhereCommand());
    // NamedCommands.registerCommand("Intake", intakeStartEnd(intakeWrist, rollers, indexer, 2));
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

    // driver.y().whileTrue(drive.sysIdDynamic(Direction.kForward));
    // driver.a().whileTrue(drive.sysIdDynamic(Direction.kReverse));

    // driver.x().whileTrue(drive.sysIdQuasistatic(Direction.kForward));
    // driver.b().whileTrue(drive.sysIdQuasistatic(Direction.kReverse));

    // driver
    //     .x()
    //     .whileTrue(
    //         AutoAlignCommands.autoAlignSourceCommand(drive, driver)
    //             .onlyIf(() -> isInSourceWing(drive))
    //             .andThen(rumbleControllers())
    //             .onlyIf(() -> !isInSourceWing(drive)));
    // test
    driver.x().whileTrue(AutoAlignShootAnywhereCommand());
    driver.y().whileTrue(shoot());
    driver.leftBumper().whileTrue(intakeDeployAndIntake(intakeWrist, rollers, indexer));

    driver
        .leftBumper()
        .onFalse(
            Commands.parallel(
                stowCommand(intakeWrist), stopIndexer(indexer), stopRollers(rollers)));

    driver.x().onFalse(Commands.parallel(stopFlywheels(), stopIndexer(indexer)));
    /*  ---------------------
      Operator Controls
    ---------------------  */

    // driver.a().whileTrue(runIndexerStartEnd());
    // driver.a().whileTrue(runIndexer(indexer, 100));
    // operator.b().whileTrue(runIndexer(indexer, -100));
    operator.x().whileTrue(runIntakeFwdCMD(rollers));
    operator.y().whileTrue(runIntakeBackCMD(rollers));
  }

  public Command stowRumble() {
    return Commands.parallel(stowCommand(intakeWrist), rumbleControllers());
  }

  public Command AutoAlignShootAnywhereCommand() {
    Logger.recordOutput("AutoAligning?", true);
    return Commands.parallel(
        AutoAlign(), rumbleIfNotSpeakerWing(), wristToSpeakerForever(), shoot());
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
        Commands.waitUntil(() -> (isPointedAtSpeaker())),
        flywheels.runFlywheelVelocity(topFlywheelVelocityRPM, bottomFlywheelVelocityRPM),
        Commands.waitUntil(() -> (flywheels.flywheelsSpunUp() && isPointedAtSpeaker())),
        runIndexer(indexer, 1000),
        Commands.waitUntil(() -> indexerBeambreak.get()),
        stopFlywheels(),
        stopIndexer(indexer));
  }

  public Command stopFlywheels() {
    return flywheels.runVoltage(0);
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

  public Command wristToSpeakerForever() {
    return shooterWrist.PIDCommandForever(this::getAngle);
  }

  private Transform2d getEstimatedTransform() {
    return new Transform2d(
        new Translation2d(
            drive.getFieldVelocity().vxMetersPerSecond * 0.02,
            drive.getFieldVelocity().vyMetersPerSecond * 0.02),
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
}
