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
    public static Alliance robotColor = null;
    // TODO: update shooter offset
    public static final Translation3d shooterOffset = new Translation3d(0, 0.10640927, 0.0055); // offset from robot coords to shooter entrance pt
  }
  public static class FieldConstants { 
    public static final Translation3d blueSpeakerCenter = new Translation3d(0.231625, 5.55818375, 2.0705635);
    public static final Translation3d redSpeakerCenter = new Translation3d(16.308315, 5.55818375, 2.0705635);
  }

  public static class UnitConstants { // isn't there a library for this
    public static final double DEG_TO_RAD = Math.PI / 180.0;
    public static final double RAD_TO_DEG = 180.0 / Math.PI;
    public static final double METRES_TO_FEETRES = Units.metersToFeet(1.0);
    public static final double FEETRES_TO_METRES = Units.feetToMeters(1.0);
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
    public static final double DEG_TO_RAD = Math.PI / 180.0;
    public static final double RAD_TO_DEG = 180.0 / Math.PI;

    public static final String CANBus = "OogaBooga";

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
    public static final double kNominal = 12.0;
    
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
    public static final int drivePeakCurrentLimit = 40;
    public static final double drivePeakCurrentDuration = 0.1;

    /*Motor Inverts Config*/
    public static final boolean angleMotorInvert = true;
    public static final boolean driveMotorInvert = false;

    /* Swerve Profiling values */
    public static final double maxSpeed = 5.5 / 4.0; // mps
    public static final double maxAngularVelocity = 450.0 * DEG_TO_RAD / 4.0; // rad per sec
    public static final boolean isFieldRelative = true;
    public static final boolean isOpenLoop = false;

    public static final double readyToShootThreshold = 0.2;

    public static enum DRIVE_STATE {
      DRIVER_CONTROL,
      SHOOTER_PREP,
      ALIGNING_TO_DPAD
    };
  }

  public static final class DPAD{

    public static enum ORIENTATION{
        FRONT,
        FRONT_RIGHT,
        RIGHT,
        DOWN_RIGHT,
        DOWN,
        DOWN_LEFT,
        LEFT,
        FRONT_LEFT,
        NON_ORIENTED
    };


    public static int value(ORIENTATION o){
      switch(o){
        case FRONT:        return 0;
        case FRONT_RIGHT:  return 45;
        case RIGHT:        return 90;
        case DOWN_RIGHT:   return 135;
        case DOWN:         return 180;
        case DOWN_LEFT:    return 0;
        case LEFT:         return 270;
        case FRONT_LEFT:   return 315;
        default:           return -1;
      }
    }
    
    // public int value(NODE_LEVEL n){
    //   switch(n){
    //     case HIGH: return 0;
    //     case MID:  return 270;
    //     case LOW:  return 180;
    //     default: return -1;
    //   }
    // }

  }

  public static final class FrontLeftModule{
    public static final int driveMotorID = 0;
    public static final int angleMotorID = 1;
    public static final int canCoderID = 2;
    // public static final double angleOffset = 352.002; //meant to be negative
    public static final double angleOffset = 0.404297 * 360.0;
    public static final double[] constants = { driveMotorID, angleMotorID, canCoderID, angleOffset };
  }

  public static final class FrontRightModule{
    public static final int driveMotorID = 3;
    public static final int angleMotorID = 4;
    public static final int canCoderID = 5;
    // public static final double angleOffset = 321.328;//meant to be negative
    public static final double angleOffset = 0.024414 * 360.0;
    public static final double[] constants = { driveMotorID, angleMotorID, canCoderID, angleOffset};
  }

  public static final class BackLeftModule{
    public static final int driveMotorID = 6;
    public static final int angleMotorID = 7;
    public static final int canCoderID = 8;
    // public static final double angleOffset = 98.525;
    public static final double angleOffset = 0.606689 * 360.0;
    public static final double[] constants = { driveMotorID, angleMotorID, canCoderID, angleOffset};
  }

  public static final class BackRightModule{
    public static final int driveMotorID = 9;
    public static final int angleMotorID = 10;
    public static final int canCoderID = 11;
    // public static final double angleOffset = 68.555;
    public static final double angleOffset = 0.268555 * 360.0;
    public static final double[] constants = { driveMotorID, angleMotorID, canCoderID, angleOffset};
  }

  public static class AutoConstants{
    public static final double MaxSpeedMetersPerSecond = 3.0;
    public static final double MaxAccelMetersPerSecondSquared = 27.0;
    public static final double MaxAngularSpeedRadiansPerSecond = 2.0 * Math.PI;
    public static final double MaxAngularAccelRadiansPerSecondSquared = 4.0 * Math.PI;

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

  public static class ClimbConstants{
    public static final int leftMotorID = 0; //fix
    public static final int rightMotorID = 6; //fix
    
    public static final double extendP = 0.1;
    public static final double extendI = 0;
    public static final double extendD = 0.01;

    public static final double balanceP = 0.01;
    public static final double balanceI = 0;
    public static final double balanceD = 0.01;

    public static final double stow = 0;
    public static final double extended = 100;
    public static final double retracted = 50;
    public static final double setPointDeadband = 3;
    public static final double balanceLimit = 20;

    public static final double gyroThreshold = 1.5; // TO DO: degrees?? idk
    public static final double errorGain = 0.05;
  }

  public static class IntakeConstants{
    public static final double kIntakeSpeed = 0.3;
    public static final int intakeMotorID = 0;

    public static final double intakeKP = 0.5;
    public static final double intakeKI = 0;
    public static final double intakeKD = 0.1;
    
    public static final double wristGearRatio = 3.58;
    public static final int throughBoreEncID = 2; 
    public static final int wristMotorID = 2;
    //WRIST ID MUST BE THE MOTOR W THE THRU ENCODER

    //public static final int encoder ID here
    //public static final int DIO_pin1 = 0;
    //public static final int DIO_pin2 = 0;
    
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
    public static final int breakerPort1 = 0;
  }

  public static class IndexerConstants{
    public static final int indexerMotorID = 1;
    public static final double kIndexerSpeed = 0.3;
    public static final int breakerPort2 = 1;

    public static final double indexerKP = 0.5;
    public static final double indexerKI = 0;
    public static final double indexerKD = 0.1;
  }

  public static class ControllerConstants{
    public static final double deadband = 0.1;
    public static final double triggerActivate = 0.8;
    
    public static final int xboxDriveID = 0;
    public static final int xboxOperatorID = 1;

    public static final int xboxLXAxis = 0;
    public static final int xboxLYAxis = 1;
    public static final int xboxRXAxis = 4;
    public static final int xboxRYAxis = 5;

    public static final int xboxLTAxis = 2;
    public static final int xboxRTAxis = 3;

    public static final int xboxA = 1;
    public static final int xboxB = 2;
    public static final int xboxX = 3;
    public static final int xboxY = 4;
    public static final int xboxLB = 5;
    public static final int xboxRB = 6;
    public static final int xboxView = 7;
    public static final int xboxMenu = 8;
    public static final int xboxLeftJoyPress = 9;
    public static final int xboxRightJoyPress = 10;
    public static final int xboxRightDPad = 11;
  }
}
