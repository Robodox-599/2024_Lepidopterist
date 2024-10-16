package frc.robot.subsystems.shooter.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterWristIO {

  @AutoLog
  public static class ShooterWristIOInputs {
    public double angleRads = 0.0;
    public double angVelocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double setpointAngleRads = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterWristIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double motorVolts) {}

  /** Returns the current distance measurement. */
  public default double getAngle() {
    return 0.0;
  }

  public default void zeroPosition() {}

  /** Sets the pivot arm voltage to 0 */
  public default void stop() {}

  /** Go to Setpoint */
  public default void goToSetpoint(double setpoint) {}

  public default void holdSetpoint(double setpoint) {}

  public default void setBrake(boolean brake) {}

  public default boolean atSetpoint() {
    return false;
  }
}
