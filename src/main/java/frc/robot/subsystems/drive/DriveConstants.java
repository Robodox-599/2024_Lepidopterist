package frc.robot.subsystems.drive;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;

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

  // Module 0 (FL) Device IDs

  public static final int Module0DriveTalon = 0;
  public static final int Module0TurnTalon = 1;
  public static final int Module0Cancoder = 2;

  // Module 1 (FR) Device IDs

  public static final int Module1DriveTalon = 3;
  public static final int Module1TurnTalon = 4;
  public static final int Module1Cancoder = 5;

  // Module 2 (BL) Device IDs

  public static final int Module2DriveTalon = 6;
  public static final int Module2TurnTalon = 7;
  public static final int Module2Cancoder = 8;

  // Module 3 (BR) Device IDs

  public static final int Module3DriveTalon = 9;
  public static final int Module3TurnTalon = 10;
  public static final int Module3Cancoder = 11;

  // Absolute Encoder Offsets

  public static final double Module0AbsoluteEncoderOffset = -0.655; // FL
  public static final double Module1AbsoluteEncoderOffset = -2.984; // FR
  public static final double Module2AbsoluteEncoderOffset = 0.690; // BL
  public static final double Module3AbsoluteEncoderOffset = -1.457; // BR

  // Supply Current Limit

  public static final int DriveMotorSupplyCurrentLimitConstant = 40;
  public static final int TurnMotorSupplyCurrentLimitConstant = 30;

  /*
   * EACH SWERVE MODULE HAS A GEAR RATIO (L1, L2, L3, OR L4)
   *
   * !! DO NOT CHANGE THESE VALUES WITHOUT FIRST CONSULTING MEER !!
   */

  // Drive and Turn Motor Gear Ratio L3

  // DRIVE GEAR RATIO
  public static final double DriveGearRatioConstant =
      ((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0));

  // TURNING GEAR RATIO
  public static final double TurnGearRatioConstant = (150.0 / 7.0);

  /*
   * THESE ARE THE CONSTANTS THAT TELL THE ROBOT ITS MAXIMUM LINEAR SPEED, TRACK WIDTH, AND DRIVE BASE RADIUS
   * TUNE THESE VALUES ACCORDINGLY.
   *
   * !! DO NOT TUNE DURING COMP OR IN COMP BRANCH !!
   *
   * IF THERE IS A NEED TO CHANGE THESE VALUES DURING COMP THEN PLEASE ASK MEER
   */

  public static final double MAX_LINEAR_SPEED = Units.feetToMeters(15.725);
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(20.75);
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(20.75);
  public static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
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

  // PATH FOLLOW TRANSLATION KP
  // public static final double pathFollowTranslationkP = 15.0;

  // PATH FOLLOW TRANSLATION KI
  // public static final double pathFollowTranslationkI = 0.15;

  // PATH FOLLOW TRANSLATION KD
  // public static final double pathFollowTranslationkD = 0.0;

  // PATH FOLLOW ROTATION KP
  // public static final double pathFollowRotationkP = 20.0;

  // PATH FOLLOW ROTATION KI
  // public static final double pathFollowRotationkI = 0.1;

  // PATH FOLLOW ROTATION KD
  // public static final double pathFollowRotationkD = 0.0;

  /*
   * PID FOR LIVE/REAL ROBOT AND REPLAY MODE ROBOT
   * TUNE THESE VALUES ACCORDINGLY
   *
   * !! DO NOT TUNE DURING COMP OR IN COMP BRANCH !!
   */

  // REAL: KS SIMPLE FEEDFORWARD
  public static final double driveRealFeedFowardkS = 0.080108;

  // REAL: KP SIMPLE FEEDFORWARD
  public static final double driveRealFeedFowardkV = 0.61952;

  // REAL: KA SIMPLE FEEDFORWARD
  public static final double driveRealFeedFowardkA = 0.019975;

  // REAL: KP DRIVE MOTOR
  public static final double driveRealFeedBackkP = 5.0;

  // REAL: KI DRIVE MOTOR
  public static final double driveRealFeedBackkI = 0.0;

  // REAL: KD DRIVE MOTOR
  public static final double driveRealFeedBackkD = 0.0;

  // REAL: KP TURN MOTOR
  public static final double turnRealFeedbackkP = 0.1;

  // REAL: KI TURN MOTOR
  public static final double turnRealFeedbackkI = 0.0;

  // REAL: KD TURN MOTOR
  public static final double turnRealFeedbackkD = 0.0;

  /*
   * PID FOR SIMULATED ROBOT
   * TUNE THESE VALUES ACCORDINGLY
   *
   * !! DO NOT TUNE DURING COMP OR IN COMP BRANCH !!
   */

  // SIM: KS SIMPLE FEEDFORWARD
  public static final double driveSimFeedFowardkS = 0.0;

  // SIM: KV SIMPLE FEEDFORWARD
  public static final double driveSimFeedFowardkV = 0.13;

  // SIM: KA SIMPLE FEEDFORWARD
  public static final double driveSimFeedFowardkA = 0.0;

  // SIM: KP DRIVE MOTOR
  public static final double driveSimFeedBackkP = 0.1;

  // SIM: KI DRIVE MOTOR
  public static final double driveSimFeedBackkI = 0.0;

  // SIM: KD DRIVE MOTOR
  public static final double driveSimFeedBackkD = 0.0;

  // SIM: KP TURN MOTOR
  public static final double turnSimFeedbackkP = 10;

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
  public static final String camera1Name = "Camera One";

  // CAMERA 1 POSE (X)
  public static final double camera1PoseX = Units.inchesToMeters(8.875);

  // CAMERA 1 POSE (Y)
  public static final double camera1PoseY = Units.inchesToMeters(10.5);

  // CAMERA 1 POSE (Z)
  public static final double camera1PoseZ = Units.inchesToMeters(8.25);

  // CAMERA 1 POSE (ROLL)
  public static final double camera1PoseRoll = Units.degreesToRadians(0);

  // CAMERA 1 POSE (PITCH)
  public static final double camera1PosePitch = Units.inchesToMeters(-28.125);

  // CAMERA 1 POSE (YAW)
  public static final double camera1PoseYaw = Units.inchesToMeters(30);

  // CAMERA 1 DISTORTION MATRIX
  public static final Matrix<N3, N3> CAMERA_ONE_MATRIX =
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

  // CAMERA 1 DISTANCE COEFFICIENTS
  public static final Matrix<N5, N1> CAMERA_ONE_DIST_COEFFS =
      MatBuilder.fill(
          Nat.N5(),
          Nat.N1(),
          0.0576413369828492,
          -0.07356597379196807,
          -6.669129885790735E-4,
          6.491281122640802E-4,
          0.03731824873787814); // Last 3 values have been truncated

  // CAMERA 2 NAME
  public static final String camera2Name = "Camera Two";

  // CAMERA 2 POSE (X)
  public static final double camera2PoseX = Units.inchesToMeters(8.875);

  // CAMERA 2 POSE (Y)
  public static final double camera2PoseY = Units.inchesToMeters(-10.5);

  // CAMERA 2 POSE (Z)
  public static final double camera2PoseZ = Units.inchesToMeters(8.25);

  // CAMERA 2 POSE (ROLL)
  public static final double camera2PoseRoll = Units.degreesToRadians(0);

  // CAMERA 2 POSE (PITCH)
  public static final double camera2PosePitch = Units.inchesToMeters(-28.125);

  // CAMERA 2 POSE (YAW)
  public static final double camera2PoseYaw = Units.inchesToMeters(-30);

  // CAMERA 2 DISTORTION MATRIX
  public static final Matrix<N3, N3> CAMERA_TWO_MATRIX =
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

  // CAMERA 2 DISTANCE COEFFICIENTS
  public static final Matrix<N5, N1> CAMERA_TWO_DIST_COEFFS =
      MatBuilder.fill(
          Nat.N5(),
          Nat.N1(),
          0.0576413369828492,
          -0.07356597379196807,
          -6.669129885790735E-4,
          6.491281122640802E-4,
          0.03731824873787814); // Last 3 values have been truncated

  /*
   * ODOMETRY FREQUENCY
   *
   * TODO: CHANGE FREQUENCY BASED ON SIM OR REAL. SET SIM TO 50 AND SET REAL TO 250.
   *
   * !! DO NOT TUNE DURING COMP OR IN COMP BRANCH !!
   */

  public static final int odomFrequency = 250;
}
