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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
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
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.vision.Vision;
// Initializing Drive Class extending SubsystemBase to make use of WPILib's command-based structure
import frc.robot.subsystems.vision.Vision.VisionConstants;
import frc.robot.subsystems.vision.VisionHelper;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOReal;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.LocalADStarAK;
import java.io.File;
import java.util.NoSuchElementException;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

public class Drive extends SubsystemBase {

  // Constants for Maximum linear speed, track width, and drive base radius

  private static final double MAX_LINEAR_SPEED =
      Units.feetToMeters(15.725065833333); // MATTHEW AND MEER PLEASE SET
  private static final double TRACK_WIDTH_X =
      Units.inchesToMeters(20.749988795006054); // MATTHEW AND MEER PLEASE SET
  private static final double TRACK_WIDTH_Y =
      Units.inchesToMeters(20.749988795006054); // MATTHEW AND MEER PLEASE SET
  private static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0); // MATTHEW AND MEER PLEASE SET
  private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
  public static final double MAX_LINEAR_ACCELERATION = 8.0;
  public static final double MAX_ANGULAR_ACCELERATION = MAX_LINEAR_ACCELERATION / DRIVE_BASE_RADIUS;
  public static final double MAX_AUTOAIM_SPEED = MAX_LINEAR_SPEED / 4;

  // Odometry lock for sychronized odometry updates

  // You lock here because odometry calculations often involve reading from and-
  // writing to shared variables that represent the state of the robot, such as its-
  // position, orientation, and velocity. These variables are updated based on sensor inputs,
  // which may be collected in different threads or subsystems. Using a lock prevents multiple
  // threads from-
  // modifying these shared variables simultaneously, which can lead to corrupted data and
  // inaccurate odometry.

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Vision[] cameras;
  public static AprilTagFieldLayout fieldTags;

  // Drive kinematics and pose estimator for position tracking
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
  Vector<N3> odoStdDevs = VecBuilder.fill(0.3, 0.3, 0.01);
  private double lastEstTimestamp = 0.0;
  private double lastOdometryUpdateTimestamp = 0.0;

  public static final Matrix<N3, N3> CAMERA_MATRIX =
      MatBuilder.fill(
          Nat.N3(),
          Nat.N3(),
          915.2126592056358,
          0.0,
          841.560216921862,
          0.0,
          913.9556728013187,
          648.2330358379004,
          0.0,
          0.0,
          1.0);
  public static final Matrix<N5, N1> DIST_COEFFS =
      MatBuilder.fill(
          Nat.N5(),
          Nat.N1(),
          0.0576413369828492,
          -0.07356597379196807,
          -6.669129885790735E-4,
          6.491281122640802E-4,
          0.03731824873787814); // Last 3 values have been truncated

  public static final VisionConstants CamConstants =
      new VisionConstants(
          "Camera One",
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-10.386),
                  Units.inchesToMeters(10.380),
                  Units.inchesToMeters(7.381)),
              new Rotation3d(
                  Units.degreesToRadians(0.0),
                  Units.degreesToRadians(-28.125),
                  Units.degreesToRadians(120))),
          CAMERA_MATRIX,
          DIST_COEFFS);

  private SwerveDriveOdometry odometry;

  // Constructor for initalizing modules and subsystems
  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      VisionIO[] visionIOs) {
    this.gyroIO = gyroIO;
    cameras = new Vision[visionIOs.length];
    // new AutoAim();
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Start threads (no-op for each if no signals have been created) // Starting threads for
    // hardware interface odometry
    PhoenixOdometryThread.getInstance().start();
    for (int i = 0; i < visionIOs.length; i++) {
      cameras[i] = new Vision(visionIOs[i]);
    }

    VisionIOSim.pose = this::getPose3d;

    SparkMaxOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner // Configuration for auto path planning
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
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
    } catch (Exception e) {
      System.err.println("Failed to load tag map");
      fieldTags = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    }

    // Configure SysId // Setup for system identification routine
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));
  }

  public static VisionIO[] createRealCameras() {
    return new VisionIO[] {new VisionIOReal(CamConstants)};
  }

  /**
   * Constructs an array of vision IOs corresponding to the simulated robot.
   *
   * @return The array of vision IOs.
   */
  public static VisionIO[] createSimCameras() {
    return new VisionIO[] {new VisionIOSim(CamConstants)};
  }

  // Regularly called method to update subsystem state

  public void periodic() {
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

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
      updateVision();
    }

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
      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void configureAutoBuilder() {
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);

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

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
    Logger.recordOutput("SwerveStates/SimCheck", RobotBase.isSimulation());
    Logger.recordOutput("SwerveStates/SimCheckReal", RobotBase.isReal());
    // Logger.recordOutput("SwerveStates/RED", Constants.isred);
    // Logger.recordOutput("SwerveStates/Blue", Constants.isblue);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

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

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
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
    return new VisionConstants[] {CamConstants};
  }
}
