package frc.robot.subsystems.indexer;

public class IndexerConstants {
  public static final int indexerMotorID = 13;
  public static final String indexerMotorCANBus = "rio";

  public static final double simkP = 1.0;
  public static final double simkI = 0.0;
  public static final double simkD = 0.0;
  public static final double simkS = 0.0;
  public static final double simkV = 0.0;
  public static final double simkA = 0.0;

  public static final double realkP = 0.1;
  public static final double realkI = 0.0;
  public static final double realkD = 0.0;
  public static final double realkS = 0.0;
  public static final double realkV = 0.0;
  public static final double realA = 0.0;
  public static final boolean EnableCurrentLimit = true;
  public static final int ContinuousCurrentLimit = 50;
  public static final int PeakCurrentLimit = 50;
  public static final double PeakCurrentDuration = 0.1;

  public static final double kIndexerSpeed = 0.35;
  public static final double kIndexerBackSpeed = -0.35;

  public static final int beakBreak2Port = 1;
  public static final double beamBreakDebounce = 0.1;

  public static final double extraIndexerTime = 0.05;
}
