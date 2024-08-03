package frc.robot.subsystems.shooter.wrist;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterWrist extends SubsystemBase {
  private final ShooterWristIOInputsAutoLogged inputs = new ShooterWristIOInputsAutoLogged();

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

  private double setpoint = 0;

  private final ShooterWristIO io;

  // Create a Mechanism2d visualization of the arm
  private MechanismLigament2d armMechanism = getArmMechanism();

  public ShooterWrist(ShooterWristIO io) {
    this.io = io;

    SmartDashboard.putData(getName(), this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);

    armMechanism.setAngle(Units.radiansToDegrees(inputs.angleRads));

    // Log Inputs
    Logger.processInputs("Wrist", inputs);
  }

  public void setBrake(boolean brake) {
    io.setBrake(brake);
  }

  @AutoLogOutput(key = "Wrist/Close")
  public boolean isVoltageClose(double setVoltage) {
    double voltageDifference = Math.abs(setVoltage - inputs.appliedVolts);
    return voltageDifference <= ShooterWristConstants.PIVOT_ARM_TOLERANCE;
  }

  public void setVoltage(double motorVolts) {
    // limit the wrist if its past the limit
    if (io.getAngle() > ShooterWristConstants.PIVOT_ARM_MAX_ANGLE && motorVolts > 0) {
      motorVolts = 0;
    } else if (io.getAngle() < ShooterWristConstants.PIVOT_ARM_MIN_ANGLE && motorVolts < 0) {
      motorVolts = 0;
    }
    io.setVoltage(motorVolts);
    isVoltageClose(motorVolts);
  }

  public void move(double speed) {
    setVoltage(speed);
  }

  public void runPID() {
    io.goToSetpoint(setpoint);
  }

  public void holdPID() {
    io.goToSetpoint(setpoint);
  }

  public void setPID(double setpoint) {
    this.setpoint = setpoint;
    Logger.recordOutput("Wrist/Setpoint", setpoint);
  }

  public void addPID(double setpointAdd) {
    this.setpoint += setpointAdd;
    this.setpoint =
        MathUtil.clamp(
            this.setpoint,
            ShooterWristConstants.PIVOT_ARM_MIN_ANGLE,
            ShooterWristConstants.PIVOT_ARM_MAX_ANGLE);

    Logger.recordOutput("Wrist/Setpoint", setpoint);
  }

  public boolean atSetpoint() {
    return Math.abs(io.getAngle() - setpoint) < ShooterWristConstants.PIVOT_ARM_PID_TOLERANCE
        && Math.abs(getVelocity()) < ShooterWristConstants.PIVOT_ARM_PID_VELOCITY_TOLERANCE;
  }

  public void setMechanism(MechanismLigament2d mechanism) {
    armMechanism = mechanism;
  }

  public Rotation2d getAngle() {
    return new Rotation2d(inputs.angleRads);
  }

  public double getVelocity() {
    return inputs.angVelocityRadsPerSec;
  }

  public Rotation2d getSetpoint() {
    return new Rotation2d(setpoint);
  }

  public MechanismLigament2d append(MechanismLigament2d mechanism) {
    return armMechanism.append(mechanism);
  }

  public MechanismLigament2d getArmMechanism() {
    return new MechanismLigament2d("Wrist", 0.4, 0, 5, new Color8Bit(Color.kAqua));
  }

  public Command PIDCommand(double setpoint) {
    return new FunctionalCommand(
        () -> setPID(setpoint), () -> runPID(), (stop) -> move(0), this::atSetpoint, this);
  }

  public Command PIDCommandForever(DoubleSupplier setpointSupplier) {
    return new FunctionalCommand(
        () -> setPID(setpointSupplier.getAsDouble()),
        () -> {
          setPID(setpointSupplier.getAsDouble());
          runPID();
        },
        (stop) -> move(0),
        () -> false,
        this);
  }

  public Command PIDCommandForever(double setpoint) {
    return new FunctionalCommand(
        () -> setPID(setpoint), () -> runPID(), (stop) -> move(0), () -> false, this);
  }

  public Command PIDHoldCommand() {
    return new FunctionalCommand(
        () -> setPID(getAngle().getRadians()),
        () -> holdPID(),
        (stop) -> move(0),
        () -> false,
        this);
  }

  public Command PIDCommand(DoubleSupplier setpointSupplier) {
    return new FunctionalCommand(
        () -> setPID(setpointSupplier.getAsDouble()),
        () -> {
          Logger.recordOutput("WristSpeakerAngle", setpointSupplier.getAsDouble());
          setPID(setpointSupplier.getAsDouble());
          runPID();
        },
        (stop) -> move(0),
        this::atSetpoint,
        this);
  }

  // Allows manual control of the pivot arm for PID tuning
  public Command ManualCommand(DoubleSupplier speedSupplier) {
    return new FunctionalCommand(
        () -> move(speedSupplier.getAsDouble()),
        () -> move(speedSupplier.getAsDouble()),
        (stop) -> move(0),
        () -> false,
        this);
  }

  public Command stop() {
    return new FunctionalCommand(
        () -> {}, () -> io.setVoltage(0), (stop) -> io.stop(), () -> false, this);
  }

  public Command bringDownCommand() {
    return new FunctionalCommand(
        () -> {},
        () -> {
          move(-1);
          setpoint = 0;
        },
        (interrupted) -> {
          move(0);
        },
        () -> {
          return io.getAngle() < 0.1;
        },
        this);
  }
}
