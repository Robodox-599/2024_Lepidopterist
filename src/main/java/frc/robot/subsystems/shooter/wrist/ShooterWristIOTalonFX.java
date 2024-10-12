package frc.robot.subsystems.shooter.wrist;

import static frc.robot.subsystems.shooter.wrist.ShooterWristConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subsystems.intake.wrist.IntakeWristConstants;
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
  private int m_WristSlot = 0;

  public ShooterWristIOTalonFX() {
    pivotMotor = new TalonFX(wristMotorID, wristMotorCANBus);

    absEncoder = new DutyCycleEncoder(3);

    var config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = 160;

    config.Slot0.kP = shooterWristRealkP;
    config.Slot0.kI = shooterWristRealkI;
    config.Slot0.kD = shooterWristRealkD;

    config.Slot0.kS = shooterWristRealkS;
    config.Slot0.kV = shooterWristRealkV;
    config.Slot0.kA = shooterWristRealkA;

    config.Slot0.kG = shooterWristRealkG;

    var motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 100;
    motionMagicConfigs.MotionMagicAcceleration = 200;

    pivotMotor.getConfigurator().apply(config);

    setBrake(true);

    absEncoder.setDistancePerRotation(360);

    pivotMotor.setPosition(Units.degreesToRotations(absEncoder.getAbsolutePosition()));

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

  /** Returns the current distance measurement. */
  @Override
  public double getAngle() {
    return (Units.rotationsToRadians(angleRads.getValueAsDouble()));
  }

  public void setDesiredWristPos(double passedInPosition) {
    m_WristSlot = passedInPosition < getAngle() ? wristRetractSlot : wristExtendSlot;

    setpoint = passedInPosition;

    MotionMagicVoltage m_request =
        new MotionMagicVoltage(setpoint)
            .withSlot(m_WristSlot)
            .withFeedForward(IntakeWristConstants.kWristFeedForward);

    pivotMotor.setControl(m_request.withPosition(setpoint));
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
}
