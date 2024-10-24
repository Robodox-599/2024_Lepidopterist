package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.Module.*;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.Module.ModuleConstants;

public final class DriveConstants {

  /*
   * THIS IS THE DEVICE ID AND NETWORK NAME THAT IS ATTACHED TO THE PIGEON 2.0 AND CANIVORE
   * TO FIND THE ID AND NETWORK NAME GO TO PHOENIX TUNER AND FLASH THE DEVICES TO ENSURE THAT THE CORRECT DEVICE IS SELECTED
   *
   * ! DO NOT CHANGE DURING COMP !
   */

  // Canbus Network Name

  public static final String canbus = "LunaDriveCANivore";

  // Gyro Device ID

  public static final int gyro = 12;

  /*
   * THESE ARE THE DEVICE IDS FOR THE MOTORS AND ENCODERS THAT ARE ATTACHED TO THE SWERVE MODULES
   * TO FIND THE IDS GO TO PHOENIX TUNER AND FLASH THE DEVICES TO ENSURE THAT THE CORRECT DEVICE IS SELECTED
   *
   * ! DO NOT CHANGE DURING COMP !
   */

  // Absolute Encoder Offsets

  public static final double Module0AbsoluteEncoderOffset = 0.106934; // FL
  public static final double Module1AbsoluteEncoderOffset = 0.472168; // FR
  public static final double Module2AbsoluteEncoderOffset = -0.105225; // BL
  public static final double Module3AbsoluteEncoderOffset = 0.220703; // BR

  public static final ModuleConstants frontLeft =
      new ModuleConstants(
          "Front Left", 0, 1, 2, Rotation2d.fromRotations(Module0AbsoluteEncoderOffset));
  public static final ModuleConstants frontRight =
      new ModuleConstants(
          "Front Right", 3, 4, 5, Rotation2d.fromRotations(Module1AbsoluteEncoderOffset));
  public static final ModuleConstants backLeft =
      new ModuleConstants(
          "Back Left", 6, 7, 8, Rotation2d.fromRotations(Module2AbsoluteEncoderOffset));
  public static final ModuleConstants backRight =
      new ModuleConstants(
          "Back Right", 9, 10, 11, Rotation2d.fromRotations(Module3AbsoluteEncoderOffset));

  /*
   * EACH SWERVE MODULE HAS A GEAR RATIO (L1, L2, L3, OR L4)
   *
   * !! DO NOT CHANGE THESE VALUES WITHOUT FIRST CONSULTING MEER !!
   */

  // Drive and Turn Motor Gear Ratio L3

  // DRIVE GEAR RATIO
  public static final double DRIVE_GEAR_RATIO = ((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0));

  // TURNING GEAR RATIO
  public static final double TURN_GEAR_RATIO = (150.0 / 7.0);

  /*
   * THESE ARE THE CONSTANTS THAT TELL THE ROBOT ITS MAXIMUM LINEAR SPEED, TRACK WIDTH, AND DRIVE BASE RADIUS
   * TUNE THESE VALUES ACCORDINGLY.
   *
   * !! DO NOT TUNE DURING COMP OR IN COMP BRANCH !!
   *
   * IF THERE IS A NEED TO CHANGE THESE VALUES DURING COMP THEN PLEASE ASK MEER
   */

  public static final double MAX_LINEAR_SPEED = Units.feetToMeters(22);
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(20.75);
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(20.75);
  public static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  public static final double MAX_ANGULAR_SPEED = (MAX_LINEAR_SPEED * 0.85) / DRIVE_BASE_RADIUS;
  public static final double MAX_LINEAR_ACCELERATION = 8.0;
  public static final double MAX_ANGULAR_ACCELERATION = MAX_LINEAR_ACCELERATION / DRIVE_BASE_RADIUS;
  public static final double MAX_AUTOAIM_SPEED = MAX_LINEAR_SPEED / 4;

  /*
   * IMPORTANT: commented out because untested on real.
   * TODO: add logic to make it pick the correct PID for sim or real.
   *
   * PID CONSTANTS FOR PATH FOLLOWING
   * TUNE THESE VALUES ACCORDINGLY
   *
   * WILL OVERRIDE DRIVE, SIM, AND REPLAY PID VALUES
   *
   * !! DO NOT TUNE DURING COMP OR IN COMP BRANCH !!
   */

  // REAL PATH FOLLOW TRANSLATION KP
  public static final double realPathFollowTranslationkP = 0.175;

  // REAL PATH FOLLOW TRANSLATION KI
  public static final double realPathFollowTranslationkI = 0.0;

  // REAL PATH FOLLOW TRANSLATION KD
  public static final double realPathFollowTranslationkD = 0.0;

  // REAL PATH FOLLOW ROTATION KP
  public static final double realPathFollowRotationkP = 3.0;

  // REAL PATH FOLLOW ROTATION KI
  public static final double realPathFollowRotationkI = 0.0;

  // REAL PATH FOLLOW ROTATION KD
  public static final double realPathFollowRotationkD = 0.0;

  // SIM PATH FOLLOWING CONSTANTS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  // SIM PATH FOLLOWING TRANSLATION KP
  public static final double simPathFollowTranslationkP = 15.0;

  // SIM PATH FOLLOW TRANSLATION KI
  public static final double simPathFollowTranslationkI = 0.15;

  // SIM PATH FOLLOW TRANSLATION KD
  public static final double simPathFollowTranslationkD = 0.0;

  // SIM PATH FOLLOW ROTATION KP
  public static final double simPathFollowRotationkP = 20.0;

  // SIM PATH FOLLOW ROTATION KI
  public static final double simPathFollowRotationkI = 0.1;

  // SIM PATH FOLLOW ROTATION KD
  public static final double simPathFollowRotationkD = 0.0;

  /*
   * PID FOR LIVE/REAL ROBOT AND REPLAY MODE ROBOT
   * TUNE THESE VALUES ACCORDINGLY
   *
   * !! DO NOT TUNE DURING COMP OR IN COMP BRANCH !!
   */

  // REAL: KS SIMPLE FEEDFORWARD
  public static final double driveRealFeedFowardkS = 0.0;

  // REAL: KP SIMPLE FEEDFORWARD
  public static final double driveRealFeedFowardkV = 0.12244;

  // REAL: KA SIMPLE FEEDFORWARD
  public static final double driveRealFeedFowardkA = 0.019679;

  // REAL: KP DRIVE MOTOR
  public static final double driveRealFeedBackkP = 0.11786;

  // REAL: KI DRIVE MOTOR
  public static final double driveRealFeedBackkI = 0.0;

  // REAL: KD DRIVE MOTOR
  public static final double driveRealFeedBackkD = 0.0;

  // REAL: KP TURN MOTOR
  public static final double turnRealFeedbackkP = 1.75;

  // REAL: KI TURN MOTOR
  public static final double turnRealFeedbackkI = 0.0;

  // REAL: KD TURN MOTOR
  public static final double turnRealFeedbackkD = 0.125;

  // REAL: KS TURN MOTOR
  public static final double turnRealFeedForwardkS = 0.1591796875;

  /*
   * PID FOR SIMULATED ROBOT
   * TUNE THESE VALUES ACCORDINGLY
   *
   * !! DO NOT TUNE DURING COMP OR IN COMP BRANCH !!
   */

  // SIM: KS SIMPLE FEEDFORWARD
  public static final double driveSimFeedFowardkS = 0.0;

  // SIM: KV SIMPLE FEEDFORWARD
  public static final double driveSimFeedFowardkV = 0.0;

  // SIM: KA SIMPLE FEEDFORWARD
  public static final double driveSimFeedFowardkA = 0.0;

  // SIM: KP DRIVE MOTOR
  public static final double driveSimFeedBackkP = 1;

  // SIM: KI DRIVE MOTOR
  public static final double driveSimFeedBackkI = 0.0;

  // SIM: KD DRIVE MOTOR
  public static final double driveSimFeedBackkD = 0;

  // SIM: KP TURN MOTOR
  public static final double turnSimFeedbackkP = 10.5;

  // SIM: KI TURN MOTOR
  public static final double turnSimFeedbackkI = 0.0;

  // SIM: KD TURN MOTOR
  public static final double turnSimFeedbackkD = 0.0;

  /*
   * CONSTANTS FOR CAMERAS
   *
   * !! DO NOT TUNE DURING COMP OR IN COMP BRANCH !!
   */

  // Camera 1 (FL) Constants

  // CAMERA 1 NAME
  public static final String camera1Name = "FL Camera";

  // CAMERA 1 POSE (X)
  public static final double camera1PoseX = Units.inchesToMeters(8.875);

  // CAMERA 1 POSE (Y)
  public static final double camera1PoseY = Units.inchesToMeters(10.5);

  // CAMERA 1 POSE (Z)
  public static final double camera1PoseZ = Units.inchesToMeters(8.25);

  // CAMERA 1 POSE (ROLL)
  public static final double camera1PoseRoll = Units.degreesToRadians(0);

  // CAMERA 1 POSE (PITCH)
  public static final double camera1PosePitch = Units.degreesToRadians(-28.125);

  // CAMERA 1 POSE (YAW)
  public static final double camera1PoseYaw = Units.degreesToRadians(30);

  // CAMERA 1 INTRINSICS MATRIX
  public static final Matrix<N3, N3> CAMERA_ONE_MATRIX =
      MatBuilder.fill(Nat.N3(), Nat.N3(), 915.14, 0.0, 914.81, 0.0, 564.33, 352.49, 0.0, 0.0, 2.0);

  // CAMERA 1 DISTORTION COEFFICIENTS
  public static final Matrix<N5, N1> CAMERA_ONE_DIST_COEFFS =
      MatBuilder.fill(
          Nat.N5(), Nat.N1(), 0.049, -0.067, -0.005, -0.011,
          0.024); // Last 3 values have been truncated

  // FR CAMERA CONSTANTS

  // CAMERA 2 NAME
  public static final String camera2Name = "FR Camera";

  // CAMERA 2 POSE (X)
  public static final double camera2PoseX = Units.inchesToMeters(8.875);

  // CAMERA 2 POSE (Y)
  public static final double camera2PoseY = Units.inchesToMeters(-10.5);

  // CAMERA 2 POSE (Z)
  public static final double camera2PoseZ = Units.inchesToMeters(8.25);

  // CAMERA 2 POSE (ROLL)
  public static final double camera2PoseRoll = Units.degreesToRadians(0);

  // CAMERA 2 POSE (PITCH)
  public static final double camera2PosePitch = Units.degreesToRadians(-28.125);

  // CAMERA 2 POSE (YAW)
  public static final double camera2PoseYaw = Units.degreesToRadians(-30);

  // CAMERA 2 INTRINSICS MATRIX
  public static final Matrix<N3, N3> CAMERA_TWO_MATRIX =
      MatBuilder.fill(Nat.N3(), Nat.N3(), 910.43, 90, 910.25, 0, 650.48, 356.69, 0.0, 0.0, 2.0);

  // CAMERA 2 DISTORTION COEFFICIENTS
  public static final Matrix<N5, N1> CAMERA_TWO_DIST_COEFFS =
      MatBuilder.fill(
          Nat.N5(), Nat.N1(), 0.067, -0.132, -0.005, -0.002,
          0.074); // Last 3 values have been truncated

  // Camera 3 (FM) Constants

  // CAMERA 3 NAME
  public static final String camera3Name = "FR Camera";

  // CAMERA 3 POSE (X)
  public static final double camera3PoseX = Units.inchesToMeters(0);

  // CAMERA 3 POSE (Y)
  public static final double camera3PoseY = Units.inchesToMeters(1);

  // CAMERA 3 POSE (Z)
  public static final double camera3PoseZ = Units.inchesToMeters(21);

  // CAMERA 3 POSE (ROLL)
  public static final double camera3PoseRoll = Units.degreesToRadians(0);

  // CAMERA 3 POSE (PITCH)
  public static final double camera3PosePitch = Units.degreesToRadians(-28);

  // CAMERA 3 POSE (YAW)
  public static final double camera3PoseYaw = Units.degreesToRadians(0);

  // CAMERA 3 INTRINSICS MATRIX
  public static final Matrix<N3, N3> CAMERA_THREE_MATRIX =
      MatBuilder.fill(Nat.N3(), Nat.N3(), 899.52, 0.0, 901.24, 0.0, 594.31, 379.39, 0.0, 0.0, 2.0);

  // CAMERA 3 DISTORTION COEFFICIENTS
  public static final Matrix<N5, N1> CAMERA_THREE_DIST_COEFFS =
      MatBuilder.fill(
          Nat.N5(), Nat.N1(), 0.048, -0.088, -0.004, -0.013,
          0.029); // Last 3 values have been truncated

  /*
   * ODOMETRY FREQUENCY
   *
   * TODO: CHANGE FREQUENCY BASED ON SIM OR REAL. SET SIM TO 50 AND SET REAL TO 250.
   *
   * !! DO NOT TUNE DURING COMP OR IN COMP BRANCH !!
   */

  public static final int odomFrequency = 250;
}
