// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // LoggedTunableNumber shooterWristLUTValue = new LoggedTunableNumber("Shooter Wrist Value ",
  // 0.0);

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }

  // public static final boolean isblue = DriverStation.getAlliance().get() == Alliance.Blue;
  // public static Optional<Alliance> isred;

  public static final boolean isReplayMode = false; // MAKE TRUE IF REPLAY MODE IS DESIRED
  public static final boolean tuningMode = true;

  public static RobotType robotType =
      isReplayMode
          ? RobotType.REPLAYBOT
          : RobotBase.isReal() ? RobotType.REALBOT : RobotType.SIMBOT;

  public enum RobotType {
    SIMBOT,

    REPLAYBOT,

    REALBOT
  }

  public static class ShooterConstants {
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
    public static final double[][] shooterLUT = {
      {0, 0, 38},
      {0.1151611524, 0, 38},
      {0.3522449123, 0, 38},
      {0.8765935905, 0, 38},
      {1.46, 0, 42},
      {1.7, 0, 38},
      {1.959336833, 0, 50.81632738},
      {2.823481946, 0, 55},
      {3.211524819, 0, 56},
      {4.258293028, 0, 57},
      {5, 0, 60}
    };

    public static class FlywheelSetpoints {
      public static final double StowSpeed = 10;
      public static final double SourceSpeed = -10;
      public static final double SpeakerSpeed = 45;
      public static final double AmpSpeed = 22;

      public static final double testFlywheelSetpoint = 45; // 15
      public static final double FlywheelCoastMargin = 5;
      public static final double FlywheelTolerance = 3.5;
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
  }

  public static class ControllerConstants {
    public static final double deadband = 0.1;
    public static final double triggerActivate = 0.8;

    public static final int xboxDriveID = 0;
    public static final int xboxOperatorID = 1;

    public static final double rumbleTime = 1; // seconds
  }
}
