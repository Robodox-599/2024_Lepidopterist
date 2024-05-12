// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
// import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.ShooterConstants.sigmoidCoefficients;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static boolean kIsReplay = false;

  public static class RobotConstants {
    public static Alliance robotColor = null;
    public static final String CANBus = "rio";
    // offset from robot coords to shooter entrance pt
    public static final Translation3d shooterOffset = new Translation3d(0, 0.46, 0.23);
    // public static final Translation3d shooterOffset = new Translation3d(0, 0.41, 0.09);
  }

  public static class FieldConstants {
    public static final Translation3d blueSpeakerCenter =
        new Translation3d(0.231625, 5.55818375, 2.0705635);
    public static final Translation3d redSpeakerCenter =
        new Translation3d(16.308315, 5.55818375, 2.0705635);

    public static Pose2d newPose(double x, double y, double theta) {
      return new Pose2d(x, y, Rotation2d.fromDegrees(theta))
          .plus(
              new Transform2d(
                  new Translation2d(-16.0 / 12.0 * UnitConstants.FEETRES_TO_METRES, 0.0)
                      .rotateBy(Rotation2d.fromDegrees(theta)),
                  Rotation2d.fromDegrees(0.0)));
    }

    public static final Pose2d[] subwooferPoses =
        new Pose2d[] {
          // newPose(15.7258562759, 4.5680435, -60.0), // R Source
          // newPose(15.7258567759, 6.552936, 60.0), // R Amp
          // newPose(15.215708, 5.5603685, 0.0), // R Center
          // newPose(0.6664485, 6.7017227241, 120.0),  // B Amp
          // newPose(1.325244, 5.5604715, 180.0), // B Center
          // newPose(0.815188224098, 4.567971, -120.0) // B Source

          new Pose2d(15.7258562759, 4.5680435, Rotation2d.fromDegrees(-60.0)), // R Source
          new Pose2d(15.7258567759, 6.552936, Rotation2d.fromDegrees(60.0)), // R Amp
          new Pose2d(15.215708, 5.5603685, Rotation2d.fromDegrees(0.0)), // R Center
          new Pose2d(0.6664485, 6.7017227241, Rotation2d.fromDegrees(120.0)), // B Amp
          new Pose2d(1.325244, 5.5604715, Rotation2d.fromDegrees(180.0)), // B Center
          new Pose2d(0.815188224098, 4.567971, Rotation2d.fromDegrees(-120.0)) // B Source
        };
    // public static final Translation2d redSubwooferAmp = new Translation2d(16.0778095,
    // 6.349736).minus(new Translation2d(16.0, Rotation2d.fromDegrees(60.0)));
    // public static final Translation2d redSubwooferCenter = new Translation2d(15.215708,
    // 5.5603685).minus(new Translation2d(16.0, Rotation2d.fromDegrees(0.0)));
    // public static final Translation2d redSubwooferSource = new Translation2d(15.7258562759,
    // 4.5680435).minus(new Translation2d(16.0, Rotation2d.fromDegrees(-60.0)));
    // public static final Translation2d blueSubwooferSource = new
    // Translation2d(0.4632355,4.771171).minus(new Translation2d(16.0,
    // Rotation2d.fromDegrees(-120.0)));
    // public static final Translation2d blueSubwooferAmp = new
    // Translation2d(0.4632485,6.34977).minus(new Translation2d(16.0,
    // Rotation2d.fromDegrees(180.0)));
    // public static final Translation2d blueSubwooferCenter = new
    // Translation2d(0.918944,5.5604715).minus(new Translation2d(16.0,
    // Rotation2d.fromDegrees(120.0)));
    public static enum SubwooferSide {
      LEFT,
      CENTER,
      RIGHT
    };
    /*0.4632485, 6.34977 amp
    0.918944, 5.5604715 center
    0.4632355, 4.771171 source

    blue^^
    16.077809, 4.7712435 source
    15.622108, 5.5603685 center
    16.0778095, 6.349736 amp

    16.077809, 4.7712435 L
    15.622108, 5.5603685 C
    16.0778095, 6.349736 R

    ^^^red */
    // inches field: https://www.desmos.com/3d/9199baf324
    // metres field: https://www.desmos.com/3d/c460283226
  }

  public static class UnitConstants {
    public static final double DEG_TO_RAD = Units.degreesToRadians(1.0);
    public static final double RAD_TO_DEG = Units.radiansToDegrees(1.0);
    public static final double METRES_TO_FEETRES = Units.metersToFeet(1.0);
    public static final double FEETRES_TO_METRES = Units.feetToMeters(1.0);
    public static final double DEG_TO_ROT = Units.degreesToRotations(1.0);
    public static final double ROT_TO_DEG = Units.rotationsToDegrees(1.0);
    public static final double kNominal = 12.0;
  }

  public static class SurfaceSpeed {

    // all surface is mps
    // TODO: decide intake and indexer percent
    public static final double shooter_rad = 0.0381; // must be in meters
    public static final double shooter_GR = 2; // must be rolloer to motor
    public static final double shooter_max_rpm = 60.5 * 60;
    public static final double shooter_transfer = 0.7863; // 0.7863

    public static final double feeder_percent = 0.6; // 0.9
    public static final double feeder_max_rpm = 90 * 60 * feeder_percent;
    public static final double feeder_rad = 0.015875; // must be in meters
    public static final double feeder_GR = 1.0 / 1.88; // must be rolloer to motor
    public static final double feeder_max_surface =
        feeder_max_rpm * feeder_GR * feeder_rad * 2 * Math.PI / 60.0;

    public static final double indexer_margin = 0; // the margin that hte indexer is slower by
    // public static final double indexer_percent = 0.5;
    public static final double indexer_rad = 0.015875; // must be in meters
    public static final double indexer_GR = 16.0 / 24.0; // must be rolloer to motor
    public static final double indexer_max_rpm = 83 * 60 * IndexerConstants.kIndexerSpeed;
    public static final double indexer_max_surface =
        indexer_max_rpm * indexer_GR * indexer_rad * 2 * Math.PI / 60.0;

    public static final double intake_margin = 0; // the margin that the intake is slower by
    // public static final double intake_percent = 0.35;
    public static final double intake_rad = 0.0254; // must be in meters
    public static final double intake_GR = 1.0 / 3.13; // must be rolloer to motor
    public static final double intake_max_rpm =
        5400 * IntakeConstants.kSpeakerIntakeSpeed; // TODO: test this
    public static final double intake_max_surface =
        intake_max_rpm * intake_GR * intake_rad * 2 * Math.PI / 60.0;
  }

  public static class IntakeConstants {
    public static final double kIntakeBackfeedSpeed = -0.5;

    public static final double kIntakeAmpScoreSpeed = -0.6;
    public static final double kSpeakerIntakeSpeed = 0.9; //
    public static final double kAmpIntakeSpeed = 1.0;
    public static final double kAutoIntakeSpeed = 1; // why
    public static final double extraIntakeTime = 2; // seconds
    public static final double backwardsIntakeTime = 0.75; // seconds
    public static final double IntakeSpeedupTime = 0.75;
    public static final double IntakeNoNoteCurrent = 30;
    public static final double maxManualRatio = 0.08;

    public static final int beamBreak1Port = 0;

    public static final int throughBoreEncID = 2;

    public static final double kWristExtendVal = 7.6;
    public static final double kWristRetractVal = 0;
    public static final double kWristAmpVal = 1.12;

    public static final double wristGearRatio = 3.58;
    public static final double kWristTolerance = 0.1; // Change

    public static class WristMotorConstants {
      // TODO: Update Motor In PHT
      public static final int wristMotorID = 18;

      public static final int wristExtendSlot = 0;
      public static final double wristExtendKP = 7;
      public static final double wristExtendKI = 0;
      public static final double wristExtendKD = 0.0;
      public static final double wristExtendKS = 0.0;
      public static final double kWristFeedForward = -0.4; // arb feedforward to account for gravity

      public static final double AbsWristP = -7;
      public static final double AbsWristI = 0;
      public static final double AbsWristD = 0;
      public static final double AbsWristFeedForward =
          -0.8; // arb feedforward to account for gravity

      public static final int wristRetractSlot = 1;
      public static final double wristRetractKP = 8;
      public static final double wristRetractKI = 0.0;
      public static final double wristRetractKD = 0.2;

      public static final double maxWristVelocity = 120;
      public static final double maxWristAccel = 240;
    }

    public static class RollerMotorConstants {
      public static final int intakeRollersMotorID = 19;
      public static final double kP = 1.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kS = 0.0;
      public static final double kV = 0.0;
      public static final double kA = 0.0;
      /*Intake Current Limit Config*/
      public static final boolean EnableCurrentLimit = true;
      public static final int ContinuousCurrentLimit = 50;
      public static final int PeakCurrentLimit = 50;
      public static final double PeakCurrentDuration = 0.1;
    }
  }

  public static class IndexerConstants {
    public static final int motorID = 13;
    public static final double indexerKP = 0.1;
    public static final double indexerKI = 0.0;
    public static final double indexerKD = 0.0;
    public static final double kIndexerSpeed = 0.35;
    public static final double kIndexerBackSpeed = -0.35;
    public static final double kIndexerSourceSpeed = -0.6;

    public static final int beakBreak2Port = 1;
    public static final double beamBreakDebounce = 0.1;

    public static final double extraIndexerTime = 0.05;
    public static final double extraSourceIndexerTime = 0.15; // seconds
    public static final double backwardsIndexerTime = 1.0; // seconds

    public static final double shootRPS = 60;
    public static final double intakeRPS = 30;
  }

  public static class LEDConstants {
    public static final int CANdle_ID = 20; // change

    public static enum LEDColor {
      CoopLED,
      AmplifyLED,
      Intook,
      Shooting,
      Amping,
      None
    };

    public static enum LEDAnimation {
      Intook,
      Shooting,
      Amping
    };
  }

  public static class ShooterConstants {
    public static final double shootTime = 0.15;
    public static final int feederMotorID = 14;
    public static final int absEncoderChannel = 4;

    public static final double cycle_speed = 0.02;
    public static final double max_manual_wrist_speed = 20;
    public static final double max_manual_ratio = max_manual_wrist_speed / (1.0 / cycle_speed);

    public static final double wristTolerance = 0.15;
    public static final double wristTestTime = 0.5;
    public static final double manualWristRatio = 0.4;

    public static final double flywheelDifferential = 5; // increase later

    public static final double shootRadius = 6; // metres (ithink)

    public static final double flywheelSourceCurrent = 30.0;
    public static final double flywheelSpikeDebounce = 0.3;

    public static final double feederRPS = 70;

    public static final InterpolatingDoubleTreeMap wristFeedForward =
        new InterpolatingDoubleTreeMap();

    public static void fillWristFeedForward() {
      wristFeedForward.put(0.0, 0.5);
      wristFeedForward.put(0.0, 0.5);
      wristFeedForward.put(0.0, 0.5);
      wristFeedForward.put(0.0, 0.5);
      wristFeedForward.put(0.0, 0.5);
      wristFeedForward.put(0.0, 0.5);
      wristFeedForward.put(0.0, 0.5);
    }

    public static class FlywheelSetpoints {
      public static final double StowSpeed = 10;
      public static final double SourceSpeed = -10;
      public static final double SpeakerSpeed = 45;
      public static final double AmpSpeed = 22;

      public static final double testFlywheelSetpoint = 45; // 15
      public static final double FlywheelCoastMargin = 5;
      public static final double FlywheelTolerance = 3.5;
    }

    public static class WristSepoints {
      public static final double minShootAngle = 8.5;
      public static final double maxShootAngle = 75;

      public static final double ampWrist = 64; // change
      public static final double sourceWrist = 63;
      public static final double testSpeakerWrist = 68; // 53 -> 51.5
      public static final double lineSetpoint = 38.8; // maybe 50?
      public static final double ampScoringDelay = 0.2;
      public static final double[] atan_radius_list = {1.8, 1, 2, 3, 4};
      public static final double[] atan_angle_list = {35.0, 30.0, 28.33, 26.67, 25.0};

      // 3.6288 for 70 deg; 3.29427 for 65 deg
    }

    public static class WristMotorConstants {
      public static final int wristID = 15;
      public static final double wristKP = 30.0;
      public static final double wristKI = 0.05;
      public static final double wristKD = 0.25;
      public static final double wristKV = 0.0;
      public static final double wristKS = 0.34;
      public static final double wristKG = 0.475; // Maybe 0.25??
      public static final double wristKA = 0.0;
      public static final double maxWristVelocity = 10.0; // motor rps
      public static final double maxWristAccel = 20.0;
    }

    public static class LeftFlywheelMotorConstants {
      public static final int leftFlywheelID = 16;
      public static final double leftFlywheelKP = 0.4;
      public static final double leftFlywheelKI = 0.0;
      public static final double leftFlywheelKD = 0.0;
      public static final double leftFlywheelKS = 0.22;
      public static final double leftFlywheelKV = 0.1268;
      public static final double leftFlywheelKA = 0.4;

      public static final boolean EnableCurrentLimit = true;
      public static final int SupplyCurrentLimit = 16;
    }

    public static class RightFlywheelMotorConstants {
      public static final int rightFlywheelID = 17;
      public static final double rightFlywheelKP = 0.4;
      public static final double rightFlywheelKI = 0.0;
      public static final double rightFlywheelKD = 0.0;
      public static final double rightFlywheelKS = 0.3;
      public static final double rightFlywheelKV = 0.124;
      public static final double rightFlywheelKA = 0.35;

      public static final boolean EnableCurrentLimit = true;
      public static final int SupplyCurrentLimit = 16;
    }

    public static class WristMathConstants {
      public static final double driverArmOffset = 56.3127;
      public static final double wristGearRatio = 24.0;

      public static final double shooterAngleToPivotAngle = 30.9; // 33.3569
      public static final double shooterConnectionLength = 7.5068; // l1
      public static final double linkageLength = 7.7504; // l2
      public static final double driveArmLength = 3.5106; // l3
    }

    public static class quinticCoefficients {
      // https://www.desmos.com/calculator/mhspzayw3n
      public static final double a = -0.0125052;
      public static final double b = 0.213676;
      public static final double c = -1.63102;
      public static final double d = 8.62777;
      public static final double e = -34.3674;
      public static final double f = 91.6852;
    }
    // TODO: update reg model please
    public static class sigmoidCoefficients {
      // https://www.desmos.com/calculator/q1noao8nzq
      public static final double minNoteVelocity = SurfaceSpeed.feeder_max_surface + 1;
      public static final double maxNoteVelocity =
          (SurfaceSpeed.shooter_max_rpm
                  * SurfaceSpeed.shooter_GR
                  * SurfaceSpeed.shooter_transfer
                  * SurfaceSpeed.shooter_rad
                  * 2.0
                  * Math.PI)
              / 60.0; // TODO: Measure this still
      public static final double Kstretch = 1.2;
      public static final double sigmoidCenter = 1.44;
    }
  }

  public static class VisionConstants {
    public static final double camHeight = 5;
    public static final double targetHeight = 5;
    public static final double camPitch = 0;
    public static final String kCameraName = "Global_Shutter_Camera";
    public static final double threshold = 0.3; // radians?
    // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
    public static final Transform3d kRobotToCam =
        new Transform3d(
            Units.inchesToMeters(3.5),
            Units.inchesToMeters(7.0),
            Units.inchesToMeters(17.5),
            new Rotation3d(0, 0, 0));

    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  }

  public static class SwerveConstants {
    public static final String CANBus = "LunaDriveCANivore";

    public static final int gyroID = 12;
    public static final boolean invertGyro = true;

    // Drivetrain Constants
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.375;

    public static final double driveGearRatio = 75.0 / 14.0;
    public static final double angleGearRatio = 150.0 / 7.0;

    public static final double wheelCircumference = Units.inchesToMeters(4.0 * Math.PI);

    public static final Translation2d frontLeft =
        new Translation2d(Units.inchesToMeters(10.375), Units.inchesToMeters(10.375));
    public static final Translation2d frontRight =
        new Translation2d(Units.inchesToMeters(10.375), -Units.inchesToMeters(10.375));
    public static final Translation2d backLeft =
        new Translation2d(-Units.inchesToMeters(10.375), Units.inchesToMeters(10.375));
    public static final Translation2d backRight =
        new Translation2d(-Units.inchesToMeters(10.375), -Units.inchesToMeters(10.375));

    public static final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(frontLeft, frontRight, backLeft, backRight);

    // Change to non-linear throttle for finer tuned movements
    public static enum Throttle {
      LINEAR,
      NONLINEAR
    };

    /*simepleMotorFeedforward gains*/
    public static final double driveKS = 0.080108;
    public static final double driveKV = 0.61952;
    public static final double driveKA = 0.019975;

    /*Angle Encoder Invert*/
    public static final AbsoluteSensorRangeValue RANGE_VALUE =
        AbsoluteSensorRangeValue.Unsigned_0To1;
    public static final SensorDirectionValue DIRECTION_VALUE =
        SensorDirectionValue.CounterClockwise_Positive;

    /*Swerve Angle Motor PID gains*/
    public static final double angleKP = 5.0;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKF = 0.0;

    /*Swerve Angle Current Limit Config*/
    public static final boolean angleEnableCurrentLimit = true;
    public static final int angleContinuousCurrentLimit = 10;
    public static final int anglePeakCurrentLimit = 20;
    public static final double anglePeakCurrentDuration = 0.1;

    /*Swerve Drive Motor PID gains*/
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /*Swerve Drive Current Limit Config*/
    public static final boolean driveEnableCurrentLimit = true;
    public static final int driveContinuousCurrentLimit = 25;
    public static final int drivePeakCurrentLimit = 30;
    public static final double drivePeakCurrentDuration = 0.1;

    /*Motor Inverts Config*/
    public static final boolean angleMotorInvert = true;
    public static final boolean driveMotorInvert = false;

    /* Swerve Profiling values */
    public static final double maxSpeed = 10.0 / 4.0; // mps
    public static final double maxAngularVelocity =
        450.0 * UnitConstants.DEG_TO_RAD / 4.0; // rad per sec
    public static final boolean isFieldRelative = true;
    public static final boolean isOpenLoop = false;

    public static final double readyToShootThreshold = 0.2;

    public static enum DRIVE_STATE {
      DRIVER_CONTROL,
      SHOOTER_PREP,
      ALIGNING_TO_DPAD
    };

    public static enum DPAD {
      UP,
      UPRIGHT,
      RIGHT,
      DOWNRIGHT,
      DOWN,
      DOWNLEFT,
      LEFT,
      UPLEFT
    };
  }

  public static final class FrontLeftModule {
    public static final int driveMotorID = 0;
    public static final int angleMotorID = 1;
    public static final int canCoderID = 2;
    public static final double angleOffset = 0.404297 * 360.0;
    public static final double[] constants = {driveMotorID, angleMotorID, canCoderID, angleOffset};
  }

  public static final class FrontRightModule {
    public static final int driveMotorID = 3;
    public static final int angleMotorID = 4;
    public static final int canCoderID = 5;
    public static final double angleOffset = 0.024414 * 360.0;
    public static final double[] constants = {driveMotorID, angleMotorID, canCoderID, angleOffset};
  }

  public static final class BackLeftModule {
    public static final int driveMotorID = 6;
    public static final int angleMotorID = 7;
    public static final int canCoderID = 8;
    public static final double angleOffset = 0.606689 * 360.0;
    public static final double[] constants = {driveMotorID, angleMotorID, canCoderID, angleOffset};
  }

  public static final class BackRightModule {
    public static final int driveMotorID = 9;
    public static final int angleMotorID = 10;
    public static final int canCoderID = 11;
    public static final double angleOffset = 0.268555 * 360.0;
    public static final double[] constants = {driveMotorID, angleMotorID, canCoderID, angleOffset};
  }

  public static class AutoConstants {
    public static final double MaxSpeedMetersPerSecond = 4.0;
    public static final double MaxAccelMetersPerSecondSquared = 7.0;
    public static final double MaxAngularSpeedRadiansPerSecond = 10.0;
    public static final double MaxAngularAccelRadiansPerSecondSquared = 8.0;

    /*Auto Swerve Drive Motor PID gains*/
    public static final double XDriveKP = 1.0;
    public static final double XDriveKD = 0.0;

    public static final PIDController XPID = new PIDController(XDriveKP, 0.0, XDriveKD);

    public static final double YDriveKP = 1.0;
    public static final double YDriveKD = 0.0;

    public static final PIDController YPID = new PIDController(YDriveKP, 0.0, YDriveKD);

    /* Auto Swerve Angle Motor PID gains*/
    public static final double AngleKP = 3.0;
    public static final double AngleKD = 0.0;

    public static final PIDController ZPID = new PIDController(AngleKP, 0.0, AngleKD);

    public static final ProfiledPIDController ThetaPIDRadians =
        new ProfiledPIDController(
            AngleKP,
            0.0,
            AngleKD,
            new TrapezoidProfile.Constraints(
                MaxAngularSpeedRadiansPerSecond, MaxAngularAccelRadiansPerSecondSquared));

    // public static final double[] AutoWristSetpoints = {61.55,50.43,52.66,50.83,
    // 66.04,50.63,65.55,58.88};
    // public static final double[] AutoFlywheelSetpoints = {8.0,8.0,8.0,8.0,
    // 8.0,8.0,8.0,8.0};
    public static final double[] AutoWristSetpointsCalc = {
      74.0, 65.0, 67.0, 66.0, 75.0, 35.0, 75.0, 72.0
    };
    public static final double[] AutoWristSetpoints = {
      65.0, 65.0, 65.0, 65.0, 65.0, 65.0, 65.0, 65.0
    };
    public static final double[] AutoFlywheelSetpointsCalc = {
      12, 22, 22, 22, 7, 22, 7, 18,
    };
    public static final double[] AutoFlywheelSetpoints = {
      sigmoidCoefficients.maxNoteVelocity - 4,
      sigmoidCoefficients.maxNoteVelocity - 4,
      sigmoidCoefficients.maxNoteVelocity - 4,
      sigmoidCoefficients.maxNoteVelocity - 4,
      sigmoidCoefficients.maxNoteVelocity - 4,
      sigmoidCoefficients.maxNoteVelocity - 4,
      sigmoidCoefficients.maxNoteVelocity - 4,
      sigmoidCoefficients.maxNoteVelocity - 4,
    };
  }

  public static class ControllerConstants {
    public static final double deadband = 0.1;
    public static final double triggerActivate = 0.8;

    public static final int xboxDriveID = 0;
    public static final int xboxOperatorID = 1;

    public static final double rumbleTime = 1; // seconds
  }
}
