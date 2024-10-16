package frc.robot.subsystems.shooter.wrist;

import static frc.robot.subsystems.shooter.wrist.ShooterWristConstants.shooterWristRealkD;
import static frc.robot.subsystems.shooter.wrist.ShooterWristConstants.shooterWristRealkI;
import static frc.robot.subsystems.shooter.wrist.ShooterWristConstants.wristMotorCANBus;
import static frc.robot.subsystems.shooter.wrist.ShooterWristConstants.wristMotorID;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import org.littletonrobotics.junction.Logger;

public class ShooterWristIOTalonFX implements ShooterWristIO {
  // Motor and Encoders
  private TalonFX pivotMotor;
  private double setpoint = 0;
  private DutyCycleEncoder absEncoder;

  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> angleVelocityRadsPerSec;
  private final StatusSignal<Double> tempCelcius;
  private final StatusSignal<Double> currentAmps;
  private final StatusSignal<Double> angleRads;
  private Boolean isHomed = false;
  private final SoftwareLimitSwitchConfigs limitConfig;

  public ShooterWristIOTalonFX() {
    pivotMotor = new TalonFX(wristMotorID, wristMotorCANBus);

    absEncoder = new DutyCycleEncoder(3);

    var config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Slot0.kP = 1.75;
    config.Slot0.kI = shooterWristRealkI;
    config.Slot0.kD = shooterWristRealkD;

    var limitConfig = new SoftwareLimitSwitchConfigs();
    limitConfig.ForwardSoftLimitEnable = false;
    limitConfig.ForwardSoftLimitThreshold = 14;
    limitConfig.ReverseSoftLimitEnable = false;
    limitConfig.ReverseSoftLimitThreshold = 0;
    this.limitConfig = limitConfig;

    pivotMotor.getConfigurator().apply(config);
    pivotMotor.getConfigurator().apply(limitConfig);

    setBrake(true);

    absEncoder.setDistancePerRotation(360);

    pivotMotor.setPosition(0);

    angleRads = pivotMotor.getPosition();
    angleVelocityRadsPerSec = pivotMotor.getVelocity();
    appliedVolts = pivotMotor.getSupplyVoltage();
    currentAmps = pivotMotor.getSupplyCurrent();
    tempCelcius = pivotMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, angleVelocityRadsPerSec, appliedVolts, currentAmps, tempCelcius, angleRads);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ShooterWristIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        angleVelocityRadsPerSec, appliedVolts, currentAmps, tempCelcius, angleRads);
    inputs.setpointAngleRads = Units.degreesToRadians(setpoint);
    inputs.angleRads = angleRads.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.tempCelsius = tempCelcius.getValueAsDouble();
    Logger.recordOutput("ShooterWrist/MotorEncoder", angleRads.getValueAsDouble());
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setVoltage(double motorVolts) {
    Logger.recordOutput("ShooterWrist/AppliedVolts", motorVolts);
    pivotMotor.setVoltage(motorVolts);
  }

  @Override
  public void zeroPosition() {
    pivotMotor.setPosition(0);
  }

  /** Returns the current distance measurement. */
  @Override
  public double getAngle() {
    return (Units.rotationsToRadians(angleRads.getValueAsDouble()));
  }

  public void setDesiredWristPos(double passedInPosition) {
    setpoint = passedInPosition;
    pivotMotor.setControl(new PositionVoltage(passedInPosition));
  }

  /** Go to Setpoint */
  @Override
  public void goToSetpoint(double go_setpoint) {
    setpoint = go_setpoint;
    setDesiredWristPos(setpoint);
  }

  @Override
  public void holdSetpoint(double setpoint) {
    goToSetpoint(setpoint);
  }

  @Override
  public void setBrake(boolean brake) {
    if (brake) {
      pivotMotor.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  @Override
  public boolean atSetpoint() {
    return Math.abs(getAngle() - setpoint) < ShooterWristConstants.shooterWristPIDTolerance;
  }

  @Override
  public boolean homeWrist(boolean force) {
    if (force) {
      isHomed = false;
    }
    if (isHomed) {
      return true;
    }
    pivotMotor.setVoltage(3);
    if (pivotMotor.getStatorCurrent().getValue() > 40) {
      pivotMotor.setVoltage(0);
      pivotMotor.setPosition(0);
      pivotMotor.getConfigurator().apply(limitConfig.withForwardSoftLimitEnable(true));
      pivotMotor.getConfigurator().apply(limitConfig.withReverseSoftLimitEnable(true));
      isHomed = true;
    }
    return isHomed;
  }
}
