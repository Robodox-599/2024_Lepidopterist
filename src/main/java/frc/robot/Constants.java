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
    public static final int wristMotorID = 40;

    public static final double kWristExtendP = 0.5;
    public static final double kWristExtendI = 0;
    public static final double kWristExtendD = 0;

    public static final double kWristRetractP = 0.1;
    public static final double kWristRetractI = 0;
    public static final double kWristRetractD = 0;

    public static final int wristExtendSlot = 0;
    public static final int wristRetractSlot = 1;

    public static final double kWristExtendVal = 30;
    public static final double kWristRetractVal = -20;

    public static final double kWristBufferZone = 2;

  }

  public static class IntakeConstants {
    public static final int intakeMotorID = 2;

    public static final double kIntakeP = 0.5;
    public static final double kIntakeI = 0;
    public static final double kIntakeD = 0.1;

    public static final double kIntakeSpeed = 0.3;
  }

  public static class BreakerConstants {
    public static final int breakerPort1 = 0;
    public static final int breakerPort2 = 1;
  }

  public static class IndexerConstants {
    public static final int indexerMotorID = 9;

    public static final double indexerSpeed = 0.3;
  }

  public static final class ControllerConstants{
    public static final double deadband = 0.1;
    public static final double triggerActivate = 0.8;
    
    public static final int xboxHaperatorID = 0;

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
