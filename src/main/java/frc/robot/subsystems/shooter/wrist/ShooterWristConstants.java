package frc.robot.subsystems.shooter.wrist;

import edu.wpi.first.math.util.Units;

public class ShooterWristConstants {
  public static final int wristMotorID = 15; // TODO: change later
  public static final String wristMotorCANBus = "rio"; // TODO: change later

  public static final double shooterWristRealkP = 0.0;
  public static final double shooterWristRealkI = 0.0;
  public static final double shooterWristRealkD = 0.0;
  public static final double shooterWristRealkS = 0.0;
  public static final double shooterWristRealkV = 0.0;
  public static final double shooterWristRealkA = 0.0;
  public static final double shooterWristRealkG = 0.0;

  public static final double shooterWristPIDTolerance =
      Units.degreesToRadians(1); // TODO: calibrate this
  public static final double shooterWristVelocityTolerance = 0.5; // TODO: calibrate this

  public static final double shooterWristMaxAngle =
      Units.degreesToRadians(90); // TODO: calibrate this
  public static final double shooterWristMinAngle =
      Units.degreesToRadians(0.0); // TODO: calibrate this

  public static final double kWristRetractVal = 0.0; // TODO: calibrate this
  public static final double kWristExtendVal = 7.95; // TODO: calibrate this
  public static final int wristExtendSlot = 0;
  public static final double wristExtendKP = 7;
  public static final double wristExtendKI = 0;
  public static final double wristExtendKD = 0.0;
  public static final double wristExtendKS = 0.0;
  public static final double kWristFeedForward = -0.4; // arb feedforward to account for gravity

  public static final int wristRetractSlot = 1;
  public static final double wristRetractKP = 8;
  public static final double wristRetractKI = 0.0;
  public static final double wristRetractKD = 0.2;

  public static final double maxWristVelocity = 100;
  public static final double maxWristAccel = 200;

  public static class ShooterWristSimConstants {
    public static final double[] kPivotSimPID = {15, 0, 0, 0}; // TODO: calibrate this

    public static final int kMotorPort = 2; // TODO: calibrate this
    public static final int kEncoderAChannel = 2; // TODO: calibrate this
    public static final int kEncoderBChannel = 3; // TODO: calibrate this

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
    public static final double kMaxAngleRads = Units.degreesToRadians(90); // TODO: calibrate this
  }
}
