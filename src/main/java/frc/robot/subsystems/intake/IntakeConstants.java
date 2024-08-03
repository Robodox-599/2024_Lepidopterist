package frc.robot.subsystems.intake;

public class IntakeConstants {

  public static final int intakeWristMotorID = 0; // TODO: fixy
  public static final String intakeWristMotorCANbus = "rio"; // TODO: fixy
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
    public static final double AbsWristFeedForward = -0.8;

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
