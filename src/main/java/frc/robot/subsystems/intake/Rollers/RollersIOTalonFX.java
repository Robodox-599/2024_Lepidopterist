package frc.robot.subsystems.intake.rollers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.intake.IntakeConstants.RollerMotorConstants;

public class RollersIOTalonFX implements RollersIO {
  private TalonFX intakeRollerMotor;
  TalonFXConfiguration intakeRollerConfig;
  private double desiredSpeed;

  public RollersIOTalonFX() {
    intakeRollerMotor =
        new TalonFX(
            RollerMotorConstants.intakeRollersMotorID,
            RollerMotorConstants.intakeRollersMotorCANbus);

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
