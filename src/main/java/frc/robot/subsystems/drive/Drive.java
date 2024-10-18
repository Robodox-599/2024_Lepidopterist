// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

// hello jaiden was here =D

package frc.robot.subsystems.drive;

import static frc.robot.Constants.robotType;
import static frc.robot.subsystems.drive.DriveConstants.CAMERA_ONE_DIST_COEFFS;
import static frc.robot.subsystems.drive.DriveConstants.CAMERA_ONE_MATRIX;
import static frc.robot.subsystems.drive.DriveConstants.CAMERA_THREE_DIST_COEFFS;
import static frc.robot.subsystems.drive.DriveConstants.CAMERA_THREE_MATRIX;
import static frc.robot.subsystems.drive.DriveConstants.CAMERA_TWO_DIST_COEFFS;
import static frc.robot.subsystems.drive.DriveConstants.CAMERA_TWO_MATRIX;
import static frc.robot.subsystems.drive.DriveConstants.DRIVE_BASE_RADIUS;
import static frc.robot.subsystems.drive.DriveConstants.MAX_ANGULAR_SPEED;
import static frc.robot.subsystems.drive.DriveConstants.MAX_LINEAR_SPEED;
import static frc.robot.subsystems.drive.DriveConstants.TRACK_WIDTH_X;
import static frc.robot.subsystems.drive.DriveConstants.TRACK_WIDTH_Y;
import static frc.robot.subsystems.drive.DriveConstants.backLeft;
import static frc.robot.subsystems.drive.DriveConstants.backRight;
import static frc.robot.subsystems.drive.DriveConstants.camera1Name;
import static frc.robot.subsystems.drive.DriveConstants.camera1PosePitch;
import static frc.robot.subsystems.drive.DriveConstants.camera1PoseRoll;
import static frc.robot.subsystems.drive.DriveConstants.camera1PoseX;
import static frc.robot.subsystems.drive.DriveConstants.camera1PoseY;
import static frc.robot.subsystems.drive.DriveConstants.camera1PoseYaw;
import static frc.robot.subsystems.drive.DriveConstants.camera1PoseZ;
import static frc.robot.subsystems.drive.DriveConstants.camera2Name;
import static frc.robot.subsystems.drive.DriveConstants.camera2PosePitch;
import static frc.robot.subsystems.drive.DriveConstants.camera2PoseRoll;
import static frc.robot.subsystems.drive.DriveConstants.camera2PoseX;
import static frc.robot.subsystems.drive.DriveConstants.camera2PoseY;
import static frc.robot.subsystems.drive.DriveConstants.camera2PoseYaw;
import static frc.robot.subsystems.drive.DriveConstants.camera2PoseZ;
import static frc.robot.subsystems.drive.DriveConstants.camera3Name;
import static frc.robot.subsystems.drive.DriveConstants.camera3PosePitch;
import static frc.robot.subsystems.drive.DriveConstants.camera3PoseRoll;
import static frc.robot.subsystems.drive.DriveConstants.camera3PoseX;
import static frc.robot.subsystems.drive.DriveConstants.camera3PoseY;
import static frc.robot.subsystems.drive.DriveConstants.camera3PoseYaw;
import static frc.robot.subsystems.drive.DriveConstants.camera3PoseZ;
import static frc.robot.subsystems.drive.DriveConstants.frontLeft;
import static frc.robot.subsystems.drive.DriveConstants.frontRight;
import static frc.robot.subsystems.drive.DriveConstants.realPathFollowRotationkD;
import static frc.robot.subsystems.drive.DriveConstants.realPathFollowRotationkI;
import static frc.robot.subsystems.drive.DriveConstants.realPathFollowRotationkP;
import static frc.robot.subsystems.drive.DriveConstants.realPathFollowTranslationkD;
import static frc.robot.subsystems.drive.DriveConstants.realPathFollowTranslationkI;
import static frc.robot.subsystems.drive.DriveConstants.realPathFollowTranslationkP;
import static frc.robot.subsystems.drive.DriveConstants.simPathFollowRotationkD;
import static frc.robot.subsystems.drive.DriveConstants.simPathFollowRotationkI;
import static frc.robot.subsystems.drive.DriveConstants.simPathFollowRotationkP;
import static frc.robot.subsystems.drive.DriveConstants.simPathFollowTranslationkD;
import static frc.robot.subsystems.drive.DriveConstants.simPathFollowTranslationkI;
import static frc.robot.subsystems.drive.DriveConstants.simPathFollowTranslationkP;

import com.google.common.collect.Streams;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.VisionConstants;
import frc.robot.subsystems.vision.VisionHelper;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.LoggedTunableNumber;
import java.io.File;
import java.util.Arrays;
import java.util.NoSuchElementException;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

public class Drive extends SubsystemBase {

  // Odometry lock for sychronized odometry updates

  // You lock here because odometry calculations often involve reading from and-
  // writing to shared variables that represent the state of the robot, such as its-
  // position, orientation, and velocity. These variables are updated based on sensor inputs,
  // which may be collected in different threads or subsystems. Using a lock prevents multiple
  // threads from-x
  // modifying these shared variables simultaneously, which can lead to corrupted data and
  // inaccurate odometry.

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Vision[] cameras;
  public static AprilTagFieldLayout fieldTags;
  private final Module[] modules; // FL, FR, BL, BR
  // Drive kinematics and pose estimator for position tracking
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Pose2d pose = new Pose2d();
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, pose);
  // Vector<N3> odoStdDevs = VecBuilder.fill(0.3, 0.3, 0.01);
  private double lastEstTimestamp = 0.0;

  public static final VisionConstants Cam1Constants =
      new VisionConstants(
          camera1Name,
          new Transform3d(
              new Translation3d(camera1PoseX, camera1PoseY, camera1PoseZ),
              new Rotation3d(camera1PoseRoll, camera1PosePitch, camera1PoseYaw)),
          CAMERA_ONE_MATRIX,
          CAMERA_ONE_DIST_COEFFS);
  public static final VisionConstants Cam2Constants =
      new VisionConstants(
          camera2Name,
          new Transform3d(
              new Translation3d(camera2PoseX, camera2PoseY, camera2PoseZ),
              new Rotation3d(camera2PoseRoll, camera2PosePitch, camera2PoseYaw)),
          CAMERA_TWO_MATRIX,
          CAMERA_TWO_DIST_COEFFS);
  public static final VisionConstants Cam3Constants =
      new VisionConstants(
          camera3Name,
          new Transform3d(
              new Translation3d(camera3PoseX, camera3PoseY, camera3PoseZ),
              new Rotation3d(camera3PoseRoll, camera3PosePitch, camera3PoseYaw)),
          CAMERA_THREE_MATRIX,
          CAMERA_THREE_DIST_COEFFS);
  // Constructor for initalizing modules and subsystems
  private static LoggedTunableNumber drivekP;
  private static LoggedTunableNumber drivekI;
  private static LoggedTunableNumber drivekD;

  private static LoggedTunableNumber turnkP;
  private static LoggedTunableNumber turnkI;
  private static LoggedTunableNumber turnkD;

  public Drive(GyroIO gyroIO, ModuleIO[] moduleIOs, VisionIO[] visionIOs) {
    this.gyroIO = gyroIO;
    cameras = new Vision[visionIOs.length];

    modules = new Module[moduleIOs.length];

    for (int i = 0; i < moduleIOs.length; i++) {
      modules[i] = new Module(moduleIOs[i]);
    }
    PhoenixOdometryThread.getInstance().start();

    for (int i = 0; i < visionIOs.length; i++) {
      cameras[i] = new Vision(visionIOs[i]);
    }

    VisionIOSim.pose = this::getPose3d;

    // SparkMaxOdometryThread.getInstance().start();

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)

    double[] driveConstantsArr;

    if (robotType == RobotType.REALBOT) {
      driveConstantsArr =
          new double[] {
            realPathFollowTranslationkP, realPathFollowTranslationkI, realPathFollowTranslationkD,
            realPathFollowRotationkP, realPathFollowRotationkI, realPathFollowRotationkD
          };

    } else {
      driveConstantsArr =
          new double[] {
            simPathFollowTranslationkP, simPathFollowTranslationkI, simPathFollowTranslationkD,
            simPathFollowRotationkP, simPathFollowRotationkI, simPathFollowRotationkD
          };
    }
    if (!DriverStation.isFMSAttached()) {
      drivekP = new LoggedTunableNumber("Translation PF P", driveConstantsArr[0]);
      drivekI = new LoggedTunableNumber("Translation PF I", driveConstantsArr[1]);
      drivekD = new LoggedTunableNumber("Translation PF D", driveConstantsArr[2]);

      turnkP = new LoggedTunableNumber("Rotation PF P", driveConstantsArr[3]);
      turnkI = new LoggedTunableNumber("Rotation PF I", driveConstantsArr[4]);
      turnkD = new LoggedTunableNumber("Rotation PF D", driveConstantsArr[5]);
    }
    final PIDConstants drivePathFollowPID =
        new PIDConstants(drivekP.get(), drivekI.get(), drivekD.get());
    final PIDConstants turnPathFollowPID =
        new PIDConstants(turnkP.get(), turnkI.get(), turnkD.get());

    // Configure AutoBuilder for PathPlanner // Configuration for auto path planning
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        this::getRobotRelativeSpeeds,
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            drivePathFollowPID,
            turnPathFollowPID,
            MAX_LINEAR_SPEED,
            DRIVE_BASE_RADIUS,
            new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    // AutoBuilder.configureHolonomic(
    //   this::getPose,
    //   this::resetOdometry,
    //   this::getRobotSpeeds,
    //   this::driveRobotRelative,
    //   new HolonomicPathFollowerConfig(
    //     new PIDConstants(14.0, 0.0, 0.0),
    //     new PIDConstants(14.0, 0.0, 0.0),
    //     AutoConstants.MaxSpeedMetersPerSecond,
    //     Math.hypot(10.375, 10.375),
    //     new ReplanningConfig()
    //     ),
    //     () -> RobotConstants.robotColor == DriverStation.Alliance.Red,
    //     // this);

    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    try {
      fieldTags =
          new AprilTagFieldLayout(
              Filesystem.getDeployDirectory()
                  .toPath()
                  .resolve("vision" + File.separator + "2024-crescendo.json"));
      System.out.println("Successfully loaded tag map");
      Logger.recordOutput("Odometry/ Tags loaded?", true);
    } catch (Exception e) {
      System.err.println("Failed to load tag map");
      fieldTags = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      Logger.recordOutput("Odometry/ Tags loaded?", false);
    }
  }

  /**
   * Constructs an array of swerve module ios corresponding to the real robot.
   *
   * @return The array of swerve module ios.
   */
  public static ModuleIO[] createTalonFXModules() {
    return new ModuleIO[] {
      new ModuleIOTalonFX(frontLeft),
      new ModuleIOTalonFX(frontRight),
      new ModuleIOTalonFX(backLeft),
      new ModuleIOTalonFX(backRight)
    };
  }

  /**
   * Constructs an array of swerve module ios corresponding to a simulated robot.
   *
   * @return The array of swerve module ios.
   */
  public static ModuleIO[] createSimModules() {
    return new ModuleIO[] {
      new ModuleIOSim("Front Left"),
      new ModuleIOSim("Front Right"),
      new ModuleIOSim("Back Left"),
      new ModuleIOSim("Back Right")
    };
  }

  public static VisionIO[] createRealCameras() {
    return new VisionIO[] {
      // new VisionIOReal(Cam1Constants),
      // new VisionIOReal(Cam2Constants),
      // new VisionIOReal(Cam3Constants)
    };
  }

  /**
   * Constructs an array of vision IOs corresponding to the simulated robot.
   *
   * @return The array of vision IOs.
   */
  public static VisionIO[] createSimCameras() {
    return new VisionIO[] {
      new VisionIOSim(Cam1Constants), new VisionIOSim(Cam2Constants), new VisionIOSim(Cam3Constants)
    };
    // return new VisionIO[]{};
  }

  // Regularly called method to update subsystem state

  public void periodic() {
    // locks odom, updates all inputs
    updateInputs();

    // calls disabled actions, to run if bot is disabled
    disabledActions();

    // apply odom update
    updateOdom();

    // Apply vision update
    updateVision();
  }

  private void disabledActions() {
    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
      updateVision();
    }
  }

  private void updateInputs() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);

    for (var module : modules) {
      module.updateInputs();
    }

    for (var camera : cameras) {
      camera.updateInputs();
      camera.processInputs();
    }

    odometryLock.unlock();

    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (var module : modules) {
      module.periodic();
    }
  }

  private void updateOdom() {
    // Update odometry // This updates based on sensor data and kinematics
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }
  }

  private void updateVision() {
    for (var camera : cameras) {
      PhotonPipelineResult result =
          new PhotonPipelineResult(camera.inputs.latency, camera.inputs.targets);
      result.setTimestampSeconds(camera.inputs.timestamp);
      boolean newResult = Math.abs(camera.inputs.timestamp - lastEstTimestamp) > 1e-5;
      try {
        var estPose = camera.update(result);
        var visionPose = estPose.get().estimatedPose;
        // Sets the pose on the sim field
        camera.setSimPose(estPose, camera, newResult);
        Logger.recordOutput("Vision/Vision Pose From " + camera.getName(), visionPose);
        Logger.recordOutput("Vision/Vision Pose2d From " + camera.getName(), visionPose.toPose2d());
        Logger.recordOutput(
            "Vision/" + camera.getName() + " pose on robot",
            visionPose.plus(camera.inputs.constants.robotToCamera()));
        poseEstimator.addVisionMeasurement(
            visionPose.toPose2d(),
            camera.inputs.timestamp,
            VisionHelper.findVisionMeasurementStdDevs(estPose.get()));
        if (newResult) lastEstTimestamp = camera.inputs.timestamp;
      } catch (NoSuchElementException e) {
      }
    }
  }

  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    Logger.recordOutput("Swerve/Target Speeds", discreteSpeeds);
    Logger.recordOutput("Swerve/Speed Error", discreteSpeeds.minus(getVelocity()));
    Logger.recordOutput(
        "Swerve/Target Chassis Speeds Field Relative",
        ChassisSpeeds.fromRobotRelativeSpeeds(discreteSpeeds, getRotation()));
    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates =
        Streams.zip(
                Arrays.stream(modules), Arrays.stream(setpointStates), (m, s) -> m.runSetpoint(s))
            .toArray(SwerveModuleState[]::new);

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public Command runVelocityCmd(Supplier<ChassisSpeeds> speeds) {
    return this.run(() -> runVelocity(speeds.get()));
  }

  /** Stops the drive. */
  public Command stopCmd() {
    return runVelocityCmd(ChassisSpeeds::new);
  }

  public Command runVelocityFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return this.runVelocityCmd(
        () -> ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), getPose().getRotation()));
  }

  public Command runVelocityTeleopFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return this.runVelocityCmd(
        () ->
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.get(),
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                    ? getPose().getRotation()
                    : getPose().getRotation().minus(Rotation2d.fromDegrees(180))));
  }

  public Command runVoltageTeleopFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return this.run(
        () -> {
          var allianceSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds.get(),
                  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                      ? getPose().getRotation()
                      : getPose().getRotation().minus(Rotation2d.fromDegrees(180)));
          // Calculate module setpoints
          ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(allianceSpeeds, 0.02);
          SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
          SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

          Logger.recordOutput("Swerve/Target Speeds", discreteSpeeds);
          Logger.recordOutput("Swerve/Field Speed Error", discreteSpeeds.minus(getVelocity()));
          Logger.recordOutput(
              "Swerve/Target Chassis Speeds Field Relative",
              ChassisSpeeds.fromRobotRelativeSpeeds(discreteSpeeds, getRotation()));

          final boolean focEnable =
              Math.sqrt(
                      Math.pow(this.getVelocity().vxMetersPerSecond, 2)
                          + Math.pow(this.getVelocity().vyMetersPerSecond, 2))
                  < MAX_LINEAR_SPEED * 0.9;

          // Send setpoints to modules

          SwerveModuleState[] optimizedSetpointStates =
              Streams.zip(
                      Arrays.stream(modules),
                      Arrays.stream(setpointStates),
                      (m, s) ->
                          m.runVoltageSetpoint(
                              new SwerveModuleState(
                                  s.speedMetersPerSecond * 12.0 / MAX_LINEAR_SPEED, s.angle),
                              focEnable))
                  .toArray(SwerveModuleState[]::new);

          // Log setpoint states
          Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
          Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
        });
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public Command stopWithXCmd() {
    return this.run(
        () -> {
          Rotation2d[] headings = new Rotation2d[4];
          for (int i = 0; i < modules.length; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
          }
          kinematics.resetHeadings(headings);
          for (int i = 0; i < modules.length; i++) {
            modules[i].runSetpoint(new SwerveModuleState(0.0, headings[i]));
          }
        });
  }

  // public Command setBrakeCommand() {
  //   return Commands.parallel(
  //       modules[0].setDaBrake(),
  //       modules[1].setDaBrake(),
  //       modules[3].setDaBrake(),
  //       modules[2].setDaBrake());
  // }

  public void resetOdometry(Pose2d pose) {
    setPose(pose);
  }

  /** Overrides the gyro angle with the specified angle in degrees. */
  public void overrideGyroAngle(double angleDegrees) {
    rawGyroRotation = Rotation2d.fromDegrees(angleDegrees);
    setPose(new Pose2d(getPose().getTranslation(), rawGyroRotation));
  }

  /** Sets the final pose for the autonomous path. */
  public void setFinalPose(Pose2d finalPose) {
    // You can implement this if you have specific actions to take with the final pose
  }

  /** Returns a command to toggle the gyro. */
  public Command toggleGyroCommand() {
    // Implement the logic to toggle the gyro if needed
    return new InstantCommand(
        () -> {
          // Logic to toggle the gyro
        });
  }

  public Command zeroGyroCommand() {
    // Implement the logic to toggle the gyro if needed
    return new InstantCommand(
        () -> {
          overrideGyroAngle(0);
          gyroIO.setYaw(0);
        });
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  @AutoLogOutput(key = "Odometry/Velocity")
  public ChassisSpeeds getVelocity() {
    var speeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            kinematics.toChassisSpeeds(
                Arrays.stream(modules).map((m) -> m.getState()).toArray(SwerveModuleState[]::new)),
            getRotation());
    return new ChassisSpeeds(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }
  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose3d getPose3d() {
    return new Pose3d(getPose());
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  @AutoLogOutput(key = "Odometry/RobotRelativeVelocity")
  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds speeds =
        kinematics.toChassisSpeeds(
            (SwerveModuleState[])
                Arrays.stream(modules).map((m) -> m.getState()).toArray(SwerveModuleState[]::new));
    return new ChassisSpeeds(
        -speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }
  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
  }

  public static VisionConstants[] getCameraConstants() {
    return new VisionConstants[] {Cam1Constants, Cam2Constants};
  }
}
