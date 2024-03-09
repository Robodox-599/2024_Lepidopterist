// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class RobotConstants {
    public static Alliance robotColor = null; // TODO: update shooter offset
    public static final String CANBus = "rio";
    // offset from robot coords to shooter entrance pt
    public static final Translation3d shooterOffset = new Translation3d(0, 0.10640927, 0.0055); 

  }

  public static class IntakeConstants{
    public static final double kIntakeSpeed = 0.3;
    public static final int intakeMotorID = 0;
    
    public static final double wristGearRatio = 3.58;
    public static final int throughBoreEncID = 4; 
    public static final int wristMotorID = 22;
    //WRIST ID MUST BE THE MOTOR W THE THRU ENCODER

    public static final int wristExtendSlot = 0;
    public static final double wristExtendKP = 0.5;
    public static final double wristExtendKI = 0;
    public static final double wristExtendKD = 0.1;

    public static final int wristRetractSlot = 1;
    public static final double wristRetractKP = 0.5;
    public static final double wristRetractKI = 0;
    public static final double wristRetractKD = 0.1;

    public static final double kWristExtendVal = 30; //Change
    public static final double kWristRetractVal = -30; //Change

    public static final double kWristBufferZone = 0.5; //Change
    public static final int breakerPort1 = 1;
  }

  public static class IndexerConstants {
    public static final int motorID = 13;
    public static final double kIndexerSpeed = 0.3;    
    public static final int breakerPort2 = 0;
    public static final double indexerKP = 0.1;
    public static final double indexerKI = 0.0;
    public static final double indexerKD = 0.0;
    public static final double setPointDeadband = 3;
  }

  public static class LEDConstants {
    public static final int CANdle_ID = 20;// change

    public static enum LEDColor {
      CoopLED,
      AmplifyLED,
      None
    };

    public static enum LEDAnimation{
      Intook,
      Shooting,
      Amping
    };
  }

  public static class Setpoints {// fix
    // public static final double start = 0;
    public static final double stow = 0;
    public static final double extended = 30;
    public static final double setPointDeadband = 3;
    public static final double PoutHigh = 0.3;
    public static final double PoutLow = -0.23;
    public static final double PoutSuper = -0.5;
  }

  public static class FieldConstants { 
    public static final Translation3d blueSpeakerCenter = new Translation3d(0.231625, 5.55818375, 2.0705635);
    public static final Translation3d redSpeakerCenter = new Translation3d(16.308315, 5.55818375, 2.0705635);
  }

  public static class UnitConstants {
    public static final double DEG_TO_RAD = Units.degreesToRadians(1.0);
    public static final double RAD_TO_DEG = Units.radiansToDegrees(1.0);
    public static final double METRES_TO_FEETRES = Units.metersToFeet(1.0);
    public static final double FEETRES_TO_METRES = Units.feetToMeters(1.0);
    public static final double kNominal = 12.0;
  }

  public static class ShooterConstants {
    public static final Translation3d blueSpeakerCenter = new Translation3d(318.2145, -56.81975, 81.321);
    public static final Translation3d redSpeakerCenter = new Translation3d(318.2145, -56.81975, 81.321);
    public static final Translation3d shooterOffset = new Translation3d(0, 0, 0);
    public static final int wristID = 15;
    public static final int leftFlywheelID = 16;
    public static final int rightFlywheelID = 17;
    public static final int feederID = 14;
    public static final double feeder_voltage = 3;
    public static final double shootRadius = 20; //metres (ithink)
    public static final double wristShootTolerance = 0.5; //deg
    public static final double flywheelShootTolerance = 80; //im ngl i have no idea what units are

    public static final double ampWrist = 2; // change
    public static final double sourceWrist = 1;
    public static final double testSpeakerWrist = 3.3;
    public static final double testFlywheelSetpoint = 2;

    public static final double minShootAngle = 16.7377; // change
    public static final double maxShootAngle = 77.5; // change
    public static final double wristError = 0.1;
    public static final double targetTime = 0.05;
    public static final double ticksToRotations = 1.0 / 8192.0;
    public static final double rotationsToAngle =1; // gear ratio stuffs
    public static final int absEncoderChannel = 2;
    public static final double minNetAngle = 16.7737;
    public static final double maxNetAngle = 70.366;

    public static final double driverArmOffset = 50.0177;
    public static final double wristGearRatio = 144/7;

    public static final double cycle_speed = 0.02;
    public static final double max_manual_wrist_speed = 20;
    public static final double max_manual_ratio = max_manual_wrist_speed/(1/cycle_speed);

    public static final double r1 = 0.0;
    public static final double r2 = 0.0;
    public static final double r3 = 0.0;
    public static final double manualWristThreshold = 0.1;

    public static final double wristKP = 4;
    public static final double wristKI = 0.0;
    public static final double wristKD = 0.0;
    public static final double wristKV = 0.0;
    public static final double wristKS = 0.0;
    public static final double wristKG = 3.39;
    public static final double wristKA = 0.0;

    public static final int DIO_pin1 = 0;
    public static final int DIO_pin2 = 1;
    public static final double maxWristVelocity = 10;
    public static final double maxWristAccel = 2;

    public static final double FlywheelMargin = 5;
    public static final double AmpSpeed = -5;
     public static final double StowSpeed = -5; // must always be equal to amp speed

    public static final double SourceSpeed = 3;
    public static final double SpeakerSpeed = -10;
   
    public static final double differential = 0;// make 50 later
    public static final double feederRuntime = 5;// sec

    public static final double shooterAngleToPivotAngle = 24.7702;
    public static final double shooterConnectionLength = 7.5068;//l1
    public static final double linkageLength = 7.7504; // l2
    public static final double driveArmLength = 3.5106;//l3


    // TODO: update sigmoid please
    public static final double noteMaxVelocity = 10;
    public static final double noteMinVelocity = 2;
    public static final double max_range = 5;
    public static final double squish = 2;
    public static final double cushion = 3;

    public static final double rightFlywheelKP = 0.2; //0.0022001
    public static final double rightFlywheelKI = 0.0;
    public static final double rightFlywheelKD = 0.0;
    public static final double rightFlywheelKS = 0.0; //0.1452;
    public static final double rightFlywheelKV = 0.0; //0.062713;
    public static final double rightFlywheelKA = 0.0; //0.0016119;

    public static final double leftFlywheelKP = 0.2; //0.020192;
    public static final double leftFlywheelKI = 0.0;
    public static final double leftFlywheelKD = 0.0;
    public static final double leftFlywheelKS = 0.0; //0.2211;
    public static final double leftFlywheelKV = 0.0; //0.062267;
    public static final double leftFlywheelKA = 0.0; //0.0030336;

    public static class quinticCoefficients{
      public static final double a = -0.00972567;
      public static final double b = 0.154702;
      public static final double c = -1.14705;
      public static final double d = 6.74407;
      public static final double e = -31.956;
      public static final double f = 90.2914;
    }
    // TODO: update reg model please
    public static class sigmoidCoefficients{
      public static final double minNoteVelocity = 2;
      public static final double maxNoteVelocity = 10;
      public static final double Kstretch = 5;
      public static final double sigmoidCenter = 2;
    }
  }

  public static class VisionConstants {
    public static final double camHeight = 5;
    public static final double targetHeight = 5;
    public static final double camPitch = 0;
    public static final String kCameraName = "Global_Shutter_Camera";
    public static final double threshold = 0.3;//radians?
    // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
    public static final Transform3d kRobotToCam = new Transform3d(Units.inchesToMeters(3.5),
                                                                  Units.inchesToMeters(7.0),
                                                                  Units.inchesToMeters(17.5),
                                                                  new Rotation3d(0, 0, 0));

    public static final AprilTagFieldLayout kTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  }

  public static class SwerveConstants{

    public static final String CANBus = "LunaDriveCANivore";

    public static final int gyroID = 12; 
    public static final boolean invertGyro = true;

    // Drivetrain Constants
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.375;

    public static final double driveGearRatio = 75.0 / 14.0;
    public static final double angleGearRatio = 150.0 / 7.0;

    public static final double wheelCircumference = Units.inchesToMeters(4.0 * Math.PI);

    public static final Translation2d frontLeft = new Translation2d(Units.inchesToMeters(10.375), Units.inchesToMeters(10.375));
    public static final Translation2d frontRight = new Translation2d(Units.inchesToMeters(10.375), -Units.inchesToMeters(10.375));
    public static final Translation2d backLeft = new Translation2d(-Units.inchesToMeters(10.375), Units.inchesToMeters(10.375));
    public static final Translation2d backRight = new Translation2d(-Units.inchesToMeters(10.375), -Units.inchesToMeters(10.375));
    
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeft, frontRight, backLeft, backRight);
    
    //Change to non-linear throttle for finer tuned movements
    public static enum Throttle{
      LINEAR,
      NONLINEAR
    };

    //Tip Correction PID (PITCH)
    public static final double pitchKP = 0.1;
    public static final double pitchKD = 0.0;

    //Tip Correction PID (ROLL) 
    public static final double rollKP = 0.1;
    public static final double rollKD = 0.0;
    public static final double timeout = 0.1;
    
    /*setting up correct units for the simepleMotorFeedforward KS gain*/
    public static final double driveKS = 0.080108;
    // public static final double driveKS = 0.1646;

    public static final double voltageKV = 0.61952;
    // public static final double voltageKV = 2.239;
    public static final double feetKV = 1.0;
    public static final double timeKV = 1.0;
    /*Setting up correct units for the simpleMotorFeedforward KV gain
    Change VoltageKV when wanting to change 
    the KV gain*/
    public static final double driveKV = voltageKV * timeKV / feetKV;

    public static final double voltageKA = 0.019975;
    // public static final double voltageKA = 0.73308;
    public static final double feetKA = 1.0;
    public static final double timeKA = 1.0;
    /*Setting up correct units for the simpleMotorFeedforward KA gain
    Change VoltageKA when wanting to change the KA gain*/
    public static final double driveKA = voltageKA * (timeKA * timeKA) / feetKA; 
    
    /*Angle Encoder Invert*/
    public static final AbsoluteSensorRangeValue RANGE_VALUE = AbsoluteSensorRangeValue.Unsigned_0To1;
    public static final SensorDirectionValue DIRECTION_VALUE = SensorDirectionValue.CounterClockwise_Positive;

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
    public static final int driveContinuousCurrentLimit = 30;
    public static final int drivePeakCurrentLimit = 35;
    public static final double drivePeakCurrentDuration = 0.1;

    /*Motor Inverts Config*/
    public static final boolean angleMotorInvert = true;
    public static final boolean driveMotorInvert = false;

    /* Swerve Profiling values */
    public static final double maxSpeed = 10.0 / 4.0; // mps
    public static final double maxAngularVelocity = 450.0 * UnitConstants.DEG_TO_RAD / 4.0; // rad per sec
    public static final boolean isFieldRelative = true;
    public static final boolean isOpenLoop = false;

    public static final double readyToShootThreshold = 0.2;

    public static enum DRIVE_STATE {
      DRIVER_CONTROL,
      SHOOTER_PREP,
      ALIGNING_TO_DPAD
    };
  }

  public static final class FrontLeftModule{
    public static final int driveMotorID = 0;
    public static final int angleMotorID = 1;
    public static final int canCoderID = 2;
    public static final double angleOffset = 0.404297 * 360.0;
    public static final double[] constants = { driveMotorID, angleMotorID, canCoderID, angleOffset };
  }

  public static final class FrontRightModule{
    public static final int driveMotorID = 3;
    public static final int angleMotorID = 4;
    public static final int canCoderID = 5;
    public static final double angleOffset = 0.024414 * 360.0;
    public static final double[] constants = { driveMotorID, angleMotorID, canCoderID, angleOffset};
  }

  public static final class BackLeftModule{
    public static final int driveMotorID = 6;
    public static final int angleMotorID = 7;
    public static final int canCoderID = 8;
    public static final double angleOffset = 0.606689 * 360.0;
    public static final double[] constants = { driveMotorID, angleMotorID, canCoderID, angleOffset};
  }

  public static final class BackRightModule{
    public static final int driveMotorID = 9;
    public static final int angleMotorID = 10;
    public static final int canCoderID = 11;
    public static final double angleOffset = 0.268555 * 360.0;
    public static final double[] constants = { driveMotorID, angleMotorID, canCoderID, angleOffset};
  }

  public static class AutoConstants{
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

    public static final PIDController ZPID = new PIDController (AngleKP, 0.0, AngleKD);

    public static final ProfiledPIDController ThetaPIDRadians = new ProfiledPIDController(
      AngleKP, 
      0.0, 
      AngleKD, 
      new TrapezoidProfile.Constraints(MaxAngularSpeedRadiansPerSecond, MaxAngularAccelRadiansPerSecondSquared));
  }

  public static class ControllerConstants{
    public static final double deadband = 0.1;
    public static final double triggerActivate = 0.8;
    
    public static final int xboxDriveID = 0;
    public static final int xboxOperatorID = 1;
  }
}