// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class WristConstants {
    public static final int wristMotorID = 8;

    public static final double kWristExtendP = 0.5;
    public static final double kWristExtendI = 0;
    public static final double kWristExtendD = 0;

    public static final double kWristRetractP = 0.1;
    public static final double kWristRetractI = 0;
    public static final double kWristRetractD = 0;

    public static final int wristExtendSlot = 0;
    public static final int wristRetractSlot = 1;

    public static final double kWristExtendVal = 30;
    public static final double kWristRetractVal = -30;

    public static final double kWristBufferZone = 0;

  }

  public static class IntakeConstants {
    
  }

  public static class BreakerConstants {
    
  }
}
