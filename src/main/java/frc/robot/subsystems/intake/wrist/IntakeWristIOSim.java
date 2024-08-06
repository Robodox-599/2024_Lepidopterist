package frc.robot.subsystems.intake.wrist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeWristIOSim implements IntakeWristIO {
  // from here
  //
  // https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armsimulation/subsystems/Arm.java
  // The P gain for the PID controller that drives this arm.

  private final DCMotor wristGearbox = DCMotor.getNEO(1);

  private final ProfiledPIDController m_controller;
  private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0, 0);

  private SingleJointedArmSim sim =
      new SingleJointedArmSim(
          wristGearbox,
          IntakeWristConstants.PivotArmSimConstants.kArmReduction,
          SingleJointedArmSim.estimateMOI(
              IntakeWristConstants.PivotArmSimConstants.kArmLength,
              IntakeWristConstants.PivotArmSimConstants.kArmMass),
          IntakeWristConstants.PivotArmSimConstants.kArmLength,
          IntakeWristConstants.PivotArmSimConstants.kMinAngleRads,
          IntakeWristConstants.PivotArmSimConstants.kMaxAngleRads,
          true, // change this to true later
          0.1);

  private final EncoderSim m_encoderSim;

  public IntakeWristIOSim() {
    m_encoderSim =
        new EncoderSim(
            new Encoder(
                IntakeWristConstants.PivotArmSimConstants.kEncoderAChannel,
                IntakeWristConstants.PivotArmSimConstants.kEncoderBChannel));

    m_encoderSim.setDistancePerPulse(
        IntakeWristConstants.PivotArmSimConstants.kArmEncoderDistPerPulse);
    m_controller =
        new ProfiledPIDController(
            IntakeWristConstants.PivotArmSimConstants.kPivotSimPID[0],
            IntakeWristConstants.PivotArmSimConstants.kPivotSimPID[1],
            IntakeWristConstants.PivotArmSimConstants.kPivotSimPID[2],
            new TrapezoidProfile.Constraints(2.45, 2.45));

    m_controller.setTolerance(0.1, 0.05);
  }

  @Override
  public void updateInputs(IntakeWristIOInputs inputs) {
    sim.update(0.02);
    inputs.angleRads = getAngle();
    inputs.angVelocityRadsPerSec = sim.getVelocityRadPerSec();
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
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
  public double getAngle() {
    return sim.getAngleRads();
  }

  @Override
  public boolean atSetpoint() {
    return m_controller.atGoal();
  }

  @Override
  public void setP(double p) {
    m_controller.setP(p);
  }

  @Override
  public void setI(double i) {
    m_controller.setI(i);
  }

  @Override
  public void setD(double d) {
    m_controller.setD(d);
  }

  @Override
  public void setkS(double kS) {
    m_feedforward = new SimpleMotorFeedforward(kS, m_feedforward.kv);
  }

  @Override
  public void setkV(double kV) {
    m_feedforward = new SimpleMotorFeedforward(m_feedforward.ks, kV);
  }

  @Override
  public double getP() {
    return m_controller.getP();
  }

  @Override
  public double getI() {
    return m_controller.getI();
  }

  @Override
  public double getD() {
    return m_controller.getD();
  }

  @Override
  public double getkS() {
    return m_feedforward.ks;
  }

  @Override
  public double getkV() {
    return m_feedforward.kv;
  }
}
