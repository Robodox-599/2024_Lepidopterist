package frc.robot.subsystems.indexer;

public class IndexerConstants {
  public static final double feedingSpeed = 0.0;
  public static final int indexerMotorID = 1;
  public static final String indexerMotorCANBus = "rio";

  public static final double kP = 1.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;

  public static final boolean EnableCurrentLimit = true;
  public static final int ContinuousCurrentLimit = 50;
  public static final int PeakCurrentLimit = 50;
  public static final double PeakCurrentDuration = 0.1;
}
