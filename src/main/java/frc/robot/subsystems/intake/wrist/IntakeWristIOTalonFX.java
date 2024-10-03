package frc.robot.subsystems.intake.wrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class IntakeWristIOTalonFX implements IntakeWristIO {
  // Motor and Encoders
  private TalonFX pivotMotor;
  private double setpoint = 0;
  private double motorEncoder;
  private int m_WristSlot = 0;

  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> angleVelocityRadsPerSec;
  private final StatusSignal<Double> tempCelcius;
  private final StatusSignal<Double> currentAmps;
  private final StatusSignal<Double> angleRads;

  public IntakeWristIOTalonFX() {
    pivotMotor =
        new TalonFX(IntakeWristConstants.wristMotorID, IntakeWristConstants.wristMotorCANBus);

    var intakeWristMotorConfig = new TalonFXConfiguration();

    intakeWristMotorConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    intakeWristMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeWristMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeWristMotorConfig.Slot0.kP = IntakeWristConstants.wristExtendKP;
    intakeWristMotorConfig.Slot0.kI = IntakeWristConstants.wristExtendKI;
    intakeWristMotorConfig.Slot0.kD = IntakeWristConstants.wristExtendKD;
    intakeWristMotorConfig.Slot0.kS = IntakeWristConstants.wristExtendKS;

    intakeWristMotorConfig.Slot1.kP = IntakeWristConstants.wristRetractKP;
    intakeWristMotorConfig.Slot1.kI = IntakeWristConstants.wristRetractKI;
    intakeWristMotorConfig.Slot1.kD = IntakeWristConstants.wristRetractKD;

    var motionMagicConfigs = intakeWristMotorConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = IntakeWristConstants.maxWristVelocity;
    motionMagicConfigs.MotionMagicAcceleration = IntakeWristConstants.maxWristAccel;

    pivotMotor.getConfigurator().apply(intakeWristMotorConfig);
    pivotMotor.setPosition(0);
    setBrake(true);

    motorEncoder = pivotMotor.getPosition().getValueAsDouble();
    angleRads = pivotMotor.getPosition();
    angleVelocityRadsPerSec = pivotMotor.getVelocity();
    appliedVolts = pivotMotor.getSupplyVoltage();
    currentAmps = pivotMotor.getSupplyCurrent();
    tempCelcius = pivotMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, angleVelocityRadsPerSec, appliedVolts, currentAmps, tempCelcius, angleRads);
    pivotMotor.optimizeBusUtilization();
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(IntakeWristIOInputs inputs) {

    BaseStatusSignal.refreshAll(
        angleVelocityRadsPerSec, appliedVolts, currentAmps, tempCelcius, angleRads);
    inputs.setpointAngleRads = setpoint;
    inputs.angleRads = angleRads.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.tempCelsius = tempCelcius.getValueAsDouble();

    Logger.recordOutput("IntakeWrist/MotorEncoder", motorEncoder);
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setVoltage(double motorVolts) {
    Logger.recordOutput("IntakeWrist/AppliedVolts", motorVolts);
    pivotMotor.setVoltage(motorVolts);
  }

  /** Returns the current distance measurement. */
  @Override
  public double getAngle() {
    return (Units.rotationsToRadians(motorEncoder));
  }

  /** Go to Setpoint */
  public void setDesiredWristPos(double passedInPosition) {
    m_WristSlot =
        passedInPosition == IntakeWristConstants.kWristExtendVal
            ? IntakeWristConstants.wristExtendSlot
            : IntakeWristConstants.wristRetractSlot;
    setpoint = passedInPosition;
    MotionMagicVoltage m_request =
        new MotionMagicVoltage(setpoint)
            .withSlot(m_WristSlot)
            .withFeedForward(IntakeWristConstants.kWristFeedForward);
    pivotMotor.setControl(m_request.withPosition(setpoint));
  }

  @Override
  public void goToSetpoint(double setpoint) {
    setDesiredWristPos(setpoint);
  }

  @Override
  public void holdSetpoint(double setpoint) {
    goToSetpoint(setpoint);
  }

  @Override
  public void setBrake(boolean brake) {
    if (brake) {
      pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  @Override
  public boolean atSetpoint() {
    return Math.abs(getAngle() - setpoint) < IntakeWristConstants.intakeWristPositionTolerance;
  }
}
