package frc.robot.subsystems.intake.rollers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.intake.rollers.RollerConstants.*;

public class RollersIOTalonFX implements RollersIO {
  private TalonFX intakeRollerMotor;
  TalonFXConfiguration intakeRollerConfig;
  private double desiredSpeed;

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
  }

  /** updates inputs from robot */
  @Override
  public void updateInputs(RollersIOInputs inputs) {
    inputs.appliedVoltage =
        (intakeRollerMotor.getClosedLoopOutput().getValueAsDouble())
            * (intakeRollerMotor.getSupplyVoltage().getValueAsDouble());
    inputs.currentAmps = new double[] {(intakeRollerMotor.getSupplyCurrent()).getValueAsDouble()};
    inputs.tempCelcius = new double[] {(intakeRollerMotor.getDeviceTemp()).getValueAsDouble()};
    inputs.velocityRadsPerSec = intakeRollerMotor.getVelocity().getValueAsDouble();
    inputs.speedSetpoint = desiredSpeed;
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
