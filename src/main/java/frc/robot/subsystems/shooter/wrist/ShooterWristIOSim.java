package frc.robot.subsystems.shooter.wrist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShooterWristIOSim implements ShooterWristIO {
  // from here
  //
  // https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armsimulation/subsystems/Arm.java
  // The P gain for the PID controller that drives this arm.

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor m_armGearbox = DCMotor.getKrakenX60(1);

  // Standard classes for controlling our arm
  private final ProfiledPIDController m_controller;
  private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0.1, 0);
  private final Encoder m_encoder;

  // Simulation classes help us simulate what's going on, including gravity.
  // This arm sim represents an arm that can travel from -75 degrees (rotated down
  // front)
  // to 255 degrees (rotated down in the back).

  private SingleJointedArmSim sim =
      new SingleJointedArmSim(
          m_armGearbox,
          ShooterWristConstants.ShooterWristSimConstants.kArmReduction,
          SingleJointedArmSim.estimateMOI(
              ShooterWristConstants.ShooterWristSimConstants.kArmLength,
              ShooterWristConstants.ShooterWristSimConstants.kArmMass),
          ShooterWristConstants.ShooterWristSimConstants.kArmLength,
          ShooterWristConstants.ShooterWristSimConstants.kMinAngleRads,
          ShooterWristConstants.ShooterWristSimConstants.kMaxAngleRads,
          true, // change this to true later
          0.1);

  private final EncoderSim m_encoderSim;

  public ShooterWristIOSim() {
    m_encoder =
        new Encoder(
            ShooterWristConstants.ShooterWristSimConstants.kEncoderAChannel,
            ShooterWristConstants.ShooterWristSimConstants.kEncoderBChannel);
    m_encoderSim = new EncoderSim(m_encoder);

    m_encoderSim.setDistancePerPulse(
        ShooterWristConstants.ShooterWristSimConstants.kArmEncoderDistPerPulse);
    m_controller =
        new ProfiledPIDController(
            ShooterWristConstants.ShooterWristSimConstants.kPivotSimPID[0],
            ShooterWristConstants.ShooterWristSimConstants.kPivotSimPID[1],
            ShooterWristConstants.ShooterWristSimConstants.kPivotSimPID[2],
            new TrapezoidProfile.Constraints(2.45, 2.45));

    m_controller.setTolerance(0.1, 0.05);
  }

  @Override
  public void updateInputs(ShooterWristIOInputs inputs) {
    sim.update(0.02);
    inputs.angleRads = getAngle();
    inputs.angVelocityRadsPerSec = sim.getVelocityRadPerSec();
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.setpointAngleRads = m_controller.getSetpoint().position;
  }

  @Override
  public void setVoltage(double motorVolts) {
    sim.setInputVoltage(motorVolts);
  }

  @Override
  public void goToSetpoint(double setpoint) {
    m_controller.setGoal(setpoint);
    // With the setpoint value we run PID control like normal
    double pidOutput = m_controller.calculate(getAngle());
    double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);

    sim.setInputVoltage(feedforwardOutput + pidOutput);
  }

  @Override
  public void zeroPosition() {
    m_encoderSim.setReset(true);
  }

  @Override
  public double getAngle() {
    return sim.getAngleRads();
  }

  @Override
  public boolean atSetpoint() {
    return m_controller.atGoal();
  }

  @Override
  public boolean homeWrist(boolean force) {
    return true;
  }
}
