package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndxerIOSim implements IndexerIO {
  private final DCMotorSim sim = new DCMotorSim(DCMotor.getNEO(1), 1.2, 2);
  private PIDController controller = new PIDController(0, 0, 0);

  private double appliedVoltage = 0.0;
  private double desiredSpeed;

  public IndxerIOSim() {
    setPIDConstants(RollerMotorConstants.kP, RollerMotorConstants.kI, RollerMotorConstants.kD);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    sim.update(0.02);
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVoltage;
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.tempCelcius = new double[] {60};
    inputs.speedSetpoint = desiredSpeed;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVoltage = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    appliedVoltage = 0.0;
    sim.setInputVoltage(0.0);
  }

  @Override
  public void setSpeed(double speed) {
    desiredSpeed = speed;
    setVoltage(controller.calculate(sim.getAngularVelocityRadPerSec(), speed));
  }
}
