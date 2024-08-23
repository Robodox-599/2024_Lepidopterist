package frc.robot.subsystems.shooter.flywheel;

public class FlywheelConstants {
  public static final int flywheelTopMotorId = 0; // TODO: assign later
  public static final int flywheelBotomMotorId = 0; // TODO: assign later

  public static final double simFlywheelFeedForwardkS = 0.1; // TODO: assign later
  public static final double simFlywheelFeedForwardkV = 0.1; // TODO: assign later
  public static final double simFlywheelFeedForwardkA = 0.1; // TODO: assign later

  public static final double simFlywheelFeedBackkP = 0.1; // TODO: assign later
  public static final double simFlywheelFeedBackkI = 0.1; // TODO: assign later
  public static final double simFlywheelFeedBackkD = 0.1; // TODO: assign later

  public static final double realFlywheelFeedForwardkS = 0.0; // TODO: assign later
  public static final double realFlywheelFeedForwardkV = 0.0; // TODO: assign later
  public static final double realFlywheelFeedForwardkA = 0.0; // TODO: assign later

  public static final double realFlywheelFeedBackkP = 0.0; // TODO: assign later
  public static final double realFlywheelFeedBackkI = 0.0; // TODO: assign later
  public static final double realFlywheelFeedBackkD = 0.0; // TODO: assign later

  public static final double FLYWHEEL_GEAR_RATIO = 1.25; // TODO: calibrate this
  public static final double topFlywheelVelocityRPM = 6000; // TODO: calibrate this
  public static final double bottomFlywheelVelocityRPM = 6000; // TODO: calibrate this
  public static final double acceptableErrorRPM = 100;
}
