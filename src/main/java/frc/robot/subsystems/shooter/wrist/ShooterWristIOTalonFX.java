package frc.robot.subsystems.shooter.wrist;

import static frc.robot.subsystems.shooter.wrist.ShooterWristConstants.shooterWristMinAngle;
import static frc.robot.subsystems.shooter.wrist.ShooterWristConstants.wristMotorCANBus;
import static frc.robot.subsystems.shooter.wrist.ShooterWristConstants.wristMotorID;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
// import frc.robot.Constants.ElectricalLayout;
import org.littletonrobotics.junction.Logger;

public class ShooterWristIOTalonFX implements ShooterWristIO {
  // Motor and Encoders
  private TalonFX pivotMotor;
  private double setpoint = 0;

  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> angleVelocityRadsPerSec;
  private final StatusSignal<Double> tempCelcius;
  private final StatusSignal<Double> currentAmps;
  private final StatusSignal<Double> angleRads;

  public ShooterWristIOTalonFX() {
    pivotMotor = new TalonFX(wristMotorID, wristMotorCANBus);

    var config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = ShooterWristConstants.shooterWristPIDReal[0];
    config.Slot0.kI = ShooterWristConstants.shooterWristPIDReal[1];
    config.Slot0.kD = ShooterWristConstants.shooterWristPIDReal[2];

    pivotMotor.getConfigurator().apply(config);

    setBrake(true);

    pivotMotor.setPosition(
        Units.degreesToRotations(shooterWristMinAngle)); // Assume we boot at hard stop

    angleRads = pivotMotor.getPosition();
    angleVelocityRadsPerSec = pivotMotor.getVelocity();
    appliedVolts = pivotMotor.getSupplyVoltage();
    currentAmps = pivotMotor.getSupplyCurrent();
    tempCelcius = pivotMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, angleVelocityRadsPerSec, appliedVolts, currentAmps, tempCelcius, angleRads);
    // configurePID();
    // configureFeedForward();
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ShooterWristIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        angleVelocityRadsPerSec, appliedVolts, currentAmps, tempCelcius, angleRads);
    inputs.setpointAngleRads = pidController.getSetpoint().position;
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

  /** Go to Setpoint */
  @Override
  public void goToSetpoint(double setpoint) {
    pidController.setGoal(setpoint);
    // With the setpoint value we run PID control like normal
    double pidOutput = MathUtil.clamp(pidController.calculate(getAngle()), -3, 3);
    double feedforwardOutput =
        feedforward.calculate(getAngle(), pidController.getSetpoint().velocity);

    Logger.recordOutput("ShooterWrist/FeedforwardOutput", feedforwardOutput);
    Logger.recordOutput("ShooterWrist/PIDOutput", pidOutput);

    Logger.recordOutput("ShooterWrist/VelocityError", pidController.getVelocityError());

    setVoltage(MathUtil.clamp(pidOutput + feedforwardOutput, -4, 4));
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
