package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.intake.IntakeConstants.RollerMotorConstants;

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
    intakeRollerConfig.Slot0.kP = RollerMotorConstants.kP;
    intakeRollerConfig.Slot0.kI = RollerMotorConstants.kI;
    intakeRollerConfig.Slot0.kD = RollerMotorConstants.kD;
    intakeRollerConfig.Slot0.kV = RollerMotorConstants.kV;
    intakeRollerConfig.Slot0.kS = RollerMotorConstants.kS;
    intakeRollerConfig.CurrentLimits.SupplyCurrentLimitEnable =
        RollerMotorConstants.EnableCurrentLimit;
    intakeRollerConfig.CurrentLimits.SupplyCurrentLimit =
        RollerMotorConstants.ContinuousCurrentLimit;
    intakeRollerConfig.CurrentLimits.SupplyCurrentThreshold = RollerMotorConstants.PeakCurrentLimit;
    intakeRollerConfig.CurrentLimits.SupplyTimeThreshold = RollerMotorConstants.PeakCurrentDuration;
  }

  /** updates inputs from robot */
  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.appliedVoltage =
        (indexerMotor.getClosedLoopOutput().getValueAsDouble())
            * (indexerMotor.getSupplyVoltage().getValueAsDouble());
    inputs.currentAmps = new double[] {(indexerMotor.getSupplyCurrent()).getValueAsDouble()};
    inputs.tempCelcius = new double[] {(indexerMotor.getDeviceTemp()).getValueAsDouble()};
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
}
