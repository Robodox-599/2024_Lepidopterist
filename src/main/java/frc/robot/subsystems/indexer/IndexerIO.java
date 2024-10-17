package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

  @AutoLog
  public static class IndexerIOInputs {
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double speedSetpoint = 0.0;
    public double currentAmps = 0.0;
    public double tempCelcius = 0.0;
  }

  /** updates inputs from robot */
  public default void updateInputs(IndexerIOInputs inputs) {}
  /** sets voltage to run motor if necessary */
  public default void setVoltage(double voltage) {}

  /** sets velocity setpoint */
  public default void stop() {
    setVoltage(0.0);
  }

  public default void setBrake(boolean brake) {}

  public default void setSpeed(double speed) {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}

  public default void prepNote() {}
}
