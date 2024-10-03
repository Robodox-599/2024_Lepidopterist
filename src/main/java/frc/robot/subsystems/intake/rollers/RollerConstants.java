package frc.robot.subsystems.intake.rollers;

import edu.wpi.first.math.util.Units;

public class RollerConstants {
  public static final int intakeRollersMotorID = 19;
  public static String intakeRollersMotorCANbus = "rio";

  public static final double kP = 1.0;
  public static final double kI = 0.0;
  public static final double kD = 1.0;
  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;
  /*Intake Current Limit Config*/
  public static final boolean EnableCurrentLimit = true; // TODO: tune
  public static final int ContinuousCurrentLimit = 50; // TODO: tune
  public static final int PeakCurrentLimit = 50; // TODO: tune
  public static final double PeakCurrentDuration = 0.1; // TODO: tune

  public static final double kSpeakerIntakeSpeed = 0.4; // TODO: tune
  public static final double kIntakeBackfeedSpeed =
      Units.rotationsPerMinuteToRadiansPerSecond(100); // TODO: tune
  public static final double extraIntakeTime = 2.0; // TODO: tune
  public static final int beamBreak1Port = 0; // TODO: tune
}
