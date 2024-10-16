package frc.robot.subsystems.shooter.flywheel;

public class FlywheelConstants {
  public static final int flywheelTopMotorId = 20; // TODO: assign later
  public static final int flywheelBotomMotorId = 14; // TODO: assign later

  public static final double simFlywheelFeedForwardkS = 0.0; // TODO: assign later
  public static final double simFlywheelFeedForwardkV = 0.0; // TODO: assign later
  public static final double simFlywheelFeedForwardkA = 0.0; // TODO: assign later
  public static final double simFlywheelFeedBackkP = 0.08; // TODO: assign later
  public static final double simFlywheelFeedBackkI = 0.0; // TODO: assign later
  public static final double simFlywheelFeedBackkD = 0.0; // TODO: assign later

  public static final double realBottomFlywheelFeedForwardkS = 0.285; // TODO: assign later
  public static final double realBottomFlywheelFeedForwardkV = 0.13129; // TODO: assign later

  public static final double realBottomFlywheelFeedBackkP = 0.25; // TODO: assign later
  public static final double realBottomFlywheelFeedBackkI = 0.0; // TODO: assign later
  public static final double realBottomFlywheelFeedBackkD = 0.0; // TODO: assign later

  public static final double realTopFlywheelFeedForwardkS = 0.285; // TODO: assign later
  public static final double realTopFlywheelFeedForwardkV = 0.13348; // TODO: assign later

  public static final double realTopFlywheelFeedBackkP = 0.2; // TODO: assign later
  public static final double realTopFlywheelFeedBackkI = 0.0; // TODO: assign later
  public static final double realTopFlywheelFeedBackkD = 0.01; // TODO: assign later

  public static final double FLYWHEEL_GEAR_RATIO = 1.25; // TODO: calibrate this
  public static final double topFlywheelVelocityRPM = 6000; // TODO: calibrate this
  public static final double bottomFlywheelVelocityRPM = 6000; // TODO: calibrate this
  public static final double acceptableErrorRPM = 100;
}
