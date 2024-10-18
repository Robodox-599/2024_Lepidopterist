package frc.robot.subsystems.intake.Rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.intake.Rollers.RollerConstants.*;

public class RollersIOTalonFX implements RollersIO {
  private TalonFX intakeRollerMotor;
  TalonFXConfiguration intakeRollerConfig;
  private double desiredSpeed;

  private final StatusSignal<Double> appliedVoltage;
  private final StatusSignal<Double> velocityRadsPerSec;
  private final StatusSignal<Double> tempCelcius;
  private final StatusSignal<Double> currentAmps;

  public RollersIOTalonFX() {
    intakeRollerMotor =
        new TalonFX(RollerConstants.intakeRollersMotorID, RollerConstants.intakeRollersMotorCANbus);

    intakeRollerConfig = new TalonFXConfiguration();
    intakeRollerConfig.Slot0.kP = RollerConstants.kP;
    intakeRollerConfig.Slot0.kI = RollerConstants.kI;
    intakeRollerConfig.Slot0.kD = RollerConstants.kD;
    intakeRollerConfig.Slot0.kV = RollerConstants.kV;
    intakeRollerConfig.Slot0.kS = RollerConstants.kS;
    intakeRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = RollerConstants.EnableCurrentLimit;
    intakeRollerConfig.CurrentLimits.SupplyCurrentLimit = RollerConstants.ContinuousCurrentLimit;
    intakeRollerConfig.CurrentLimits.SupplyCurrentThreshold = RollerConstants.PeakCurrentLimit;
    intakeRollerConfig.CurrentLimits.SupplyTimeThreshold = RollerConstants.PeakCurrentDuration;

    appliedVoltage = (intakeRollerMotor.getSupplyVoltage());
    velocityRadsPerSec = intakeRollerMotor.getVelocity();
    tempCelcius = intakeRollerMotor.getDeviceTemp();
    currentAmps = intakeRollerMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, appliedVoltage, velocityRadsPerSec, tempCelcius, currentAmps);
    // optimize comms between Talons and CAN bus
    intakeRollerMotor.getConfigurator().apply(intakeRollerConfig);
    intakeRollerMotor.optimizeBusUtilization();
  }

  /** updates inputs from robot */
  @Override
  public void updateInputs(RollersIOInputs inputs) {
    BaseStatusSignal.refreshAll(appliedVoltage, velocityRadsPerSec, tempCelcius, currentAmps);

    inputs.speedSetpoint = desiredSpeed;
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.velocityRadsPerSec = velocityRadsPerSec.getValueAsDouble();
    inputs.tempCelcius = tempCelcius.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  /** sets voltage to run motor if necessary */
  @Override
  public void setVoltage(double voltage) {
    intakeRollerMotor.setVoltage(voltage);
  }

  /** sets brake mode to stop */
  @Override
  public void setBrake(boolean brake) {
    intakeRollerMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  /** sets speed of motor */
  @Override
  public void setSpeed(double speed) {
    desiredSpeed = speed;
    intakeRollerMotor.set(speed);
  }
}
