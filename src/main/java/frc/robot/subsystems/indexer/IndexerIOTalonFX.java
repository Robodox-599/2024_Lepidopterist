package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.IndexerConstants;

public class IndexerIOTalonFX implements IndexerIO {
  private TalonFX indexerMotor;
  TalonFXConfiguration intakeRollerConfig;
  private double desiredSpeed;

  public IndexerIOTalonFX() {
    indexerMotor =
        new TalonFX(
            IndexerConstants.motorID,
            frc.robot.subsystems.indexer.IndexerConstants.indexerMotorCANBus);
    intakeRollerConfig = new TalonFXConfiguration();
    intakeRollerConfig.Slot0.kP = realkP;
    intakeRollerConfig.Slot0.kI = realkI;
    intakeRollerConfig.Slot0.kD = realkD;
    intakeRollerConfig.Slot0.kV = realkV;
    intakeRollerConfig.Slot0.kS = realkS;
    intakeRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
    intakeRollerConfig.CurrentLimits.SupplyCurrentLimit = ContinuousCurrentLimit;
    intakeRollerConfig.CurrentLimits.SupplyCurrentThreshold = PeakCurrentLimit;
    intakeRollerConfig.CurrentLimits.SupplyTimeThreshold = PeakCurrentDuration;
  }

  /** updates inputs from robot */
  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.appliedVoltage =
        (indexerMotor.getClosedLoopOutput().getValueAsDouble())
            * (indexerMotor.getSupplyVoltage().getValueAsDouble());
    inputs.currentAmps = indexerMotor.getSupplyCurrent().getValueAsDouble();
    inputs.tempCelcius = (indexerMotor.getDeviceTemp()).getValueAsDouble();
    inputs.velocityRadsPerSec = indexerMotor.getVelocity().getValueAsDouble();
    inputs.speedSetpoint = desiredSpeed;
  }

  /** sets voltage to run motor if necessary */
  @Override
  public void setVoltage(double voltage) {
    indexerMotor.setVoltage(voltage);
  }

  /** sets brake mode to stop */
  @Override
  public void setBrake(boolean brake) {
    indexerMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  /** sets speed of motor */
  @Override
  public void setSpeed(double speed) {
    desiredSpeed = speed;
    indexerMotor.set(speed);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    indexerMotor.getConfigurator().apply(config);
  }
}
