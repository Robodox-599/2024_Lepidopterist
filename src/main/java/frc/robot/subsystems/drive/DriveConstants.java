package frc.robot.subsystems.drive;

public final class DriveConstants {

  // Canbus Network Name
  // You must specify the canbus name that the devices are on

  public static final String canbus = "LunaDriveCANivore";

  // Module 0 Device IDs

  public static final int Module0DriveTalon = 0;
  public static final int Module0TurnTalon = 1;
  public static final int Module0Cancoder = 2;

  // Module 1 Device IDs

  public static final int Module1DriveTalon = 3;
  public static final int Module1TurnTalon = 4;
  public static final int Module1Cancoder = 5;

  // Module 2 Device IDs

  public static final int Module2DriveTalon = 6;
  public static final int Module2TurnTalon = 7;
  public static final int Module2Cancoder = 8;

  // Module 3 Device IDs

  public static final int Module3DriveTalon = 9;
  public static final int Module3TurnTalon = 10;
  public static final int Module3Cancoder = 11;

  // Absolute Encoder Offsets

  public static final double Module0AbsoluteEncoderOffset = -0.655;
  public static final double Module1AbsoluteEncoderOffset = -2.984;
  public static final double Module2AbsoluteEncoderOffset = 0.690;
  public static final double Module3AbsoluteEncoderOffset = -1.457;

  // Supply Current Limit

  public static final int DriveMotorSupplyCurrentLimitConstant = 40;
  public static final int TurnMotorSupplyCurrentLimitConstant = 30;

  // Drive and Turn Motor Gear Ratio L3

  public static final double DriveGearRatioConstant =
      ((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0));
  public static final double TurnGearRatioConstant = (150.0 / 7.0);
}
