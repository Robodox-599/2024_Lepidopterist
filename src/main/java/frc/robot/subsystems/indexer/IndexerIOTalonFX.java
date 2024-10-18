package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.ContinuousCurrentLimit;
import static frc.robot.subsystems.indexer.IndexerConstants.EnableCurrentLimit;
import static frc.robot.subsystems.indexer.IndexerConstants.PeakCurrentDuration;
import static frc.robot.subsystems.indexer.IndexerConstants.PeakCurrentLimit;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IndexerIOTalonFX implements IndexerIO {
  private TalonFX indexerMotor;
  TalonFXConfiguration indexerConfig;
  private double desiredSpeed;

  private final StatusSignal<Double> appliedVoltage;
  private final StatusSignal<Double> velocityRadsPerSec;
  private final StatusSignal<Double> tempCelcius;
  private final StatusSignal<Double> currentAmps;

  public IndexerIOTalonFX() {
    indexerMotor =
        new TalonFX(IndexerConstants.indexerMotorID, IndexerConstants.indexerMotorCANBus);
    indexerConfig = new TalonFXConfiguration();
    indexerConfig.Slot0.kP = 0.75;
    indexerConfig.Slot0.kI = 0.0;
    indexerConfig.Slot0.kD = 0.0465;
    indexerConfig.Slot0.kS = 0.336;
    indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
    indexerConfig.CurrentLimits.SupplyCurrentLimit = ContinuousCurrentLimit;
    indexerConfig.CurrentLimits.SupplyCurrentThreshold = PeakCurrentLimit;
    indexerConfig.CurrentLimits.SupplyTimeThreshold = PeakCurrentDuration;

    appliedVoltage = (indexerMotor.getSupplyVoltage());
    velocityRadsPerSec = indexerMotor.getVelocity();
    tempCelcius = indexerMotor.getDeviceTemp();
    currentAmps = indexerMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, appliedVoltage, velocityRadsPerSec, tempCelcius, currentAmps);
    // optimize comms between Talons and CAN bus

    indexerMotor.getConfigurator().apply(indexerConfig);

    indexerMotor.optimizeBusUtilization();
  }

  /** updates inputs from robot */
  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    BaseStatusSignal.refreshAll(appliedVoltage, velocityRadsPerSec, tempCelcius, currentAmps);

    inputs.speedSetpoint = desiredSpeed;
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.velocityRadsPerSec = velocityRadsPerSec.getValueAsDouble();
    inputs.tempCelcius = tempCelcius.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
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
  public void prepNote() {
    indexerMotor.setControl(
        new PositionVoltage((indexerMotor.getPosition().getValueAsDouble() + 1))
            .withVelocity(0.25));
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
