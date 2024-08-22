package frc.robot.subsystems.intake.wrist;

import edu.wpi.first.math.util.Units;

public class IntakeWristConstants {
  public static final int wristMotorID = 18; // TODO: change later
  public static final String wristMotorCANBus = "LunaDriveCANivore"; // TODO: change later

  public static final double[] intakeWristPIDReal = {3.6, 0, 0, 0.01}; // TODO: calibrate this
  public static final double[] intakeWristFFReal = {0, 0.45, 0, 0}; // TODO: calibrate this

  public static final double intakeWristPositionTolerance =
      Units.degreesToRadians(1); // TODO: calibrate this
  public static final double intakeWristVelocityTolerance = 0.5; // TODO: calibrate this

  public static final double intakeWristMaxAngle =
      Units.degreesToRadians(0.0); // TODO: calibrate this
  public static final double intakeWristMinAngle =
      Units.degreesToRadians(0.0); // TODO: calibrate this

  public static final double kWristRetractVal = 0.0; // TODO: calibrate this
  public static final double kWristExtendVal = 0.0; // TODO: calibrate this

  public static class IntakeWristSimConstants {
    public static final double[] kPivotSimPID = {15, 0, 0, 0}; // TODO: calibrate this

    public static final int kMotorPort = 2; // TODO: calibrate this
    public static final int kEncoderAChannel = 4; // TODO: calibrate this
    public static final int kEncoderBChannel = 5; // TODO: calibrate this

    // The P gain for the PID controller that drives this arm.
    public static final double kDefaultArmSetpointDegrees =
        Units.degreesToRadians(75.0); // TODO: calibrate this

    // distance per pulse = (angle per revolution) / (pulses per revolution)
    // = (2 * PI rads) / (4096 pulses)
    public static final double kArmEncoderDistPerPulse = 1 / 4096; // TODO: calibrate this

    public static final double kArmReduction = 200; // TODO: calibrate this
    public static final double kArmMass = 10.0; // Kilograms // TODO: calibrate this
    public static final double kArmLength = Units.inchesToMeters(20); // TODO: calibrate this
    public static final double kMinAngleRads = Units.degreesToRadians(0); // TODO: calibrate this
    public static final double kMaxAngleRads = Units.degreesToRadians(180); // TODO: calibrate this
  }
}
