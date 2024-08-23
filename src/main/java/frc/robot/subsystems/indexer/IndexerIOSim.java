package frc.robot.subsystems.indexer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
  private final DCMotorSim indexerMotorSim = new DCMotorSim(DCMotor.getKrakenX60(1), 1, 0.04);
  private PIDController controller = new PIDController(0, 0, 0);

  private double appliedVoltage;
  private double desiredSpeed;

  public IndexerIOSim() {
    configurePID(IndexerConstants.simkP, IndexerConstants.simkI, IndexerConstants.simkD);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    indexerMotorSim.update(0.02);
    inputs.velocityRadsPerSec = indexerMotorSim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVoltage;
    inputs.currentAmps = indexerMotorSim.getCurrentDrawAmps();
    inputs.tempCelcius = 60;
    inputs.speedSetpoint = desiredSpeed;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVoltage = volts;
    indexerMotorSim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    appliedVoltage = 0.0;
    indexerMotorSim.setInputVoltage(0.0);
  }

  @Override
  public void setSpeed(double speed) {
    desiredSpeed = speed;
    setVoltage(controller.calculate(indexerMotorSim.getAngularVelocityRadPerSec(), speed));
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    controller.setPID(kP, kI, kD);
  }
}
