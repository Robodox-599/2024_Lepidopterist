package frc.robot.subsystems.intake.wrist;

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

public class IntakeWrist extends SubsystemBase {
  private final IntakeWristIOInputsAutoLogged inputs = new IntakeWristIOInputsAutoLogged();
  private double motorEncoder;
  private double m_DesiredWristPos;
  private int m_WristSlot;
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

  private double setpoint = 0;

  private final IntakeWristIO io;

  // Create a Mechanism2d visualization of the arm
  private MechanismLigament2d wristMechanism = getArmMechanism();

  public IntakeWrist(IntakeWristIO io) {
    this.io = io;
    SmartDashboard.putData(getName(), this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("PivotArm", inputs);

    wristMechanism.setAngle(Units.radiansToDegrees(inputs.angleRads));

    // Log Inputs
    Logger.processInputs("PivotArm", inputs);
  }

  public void setBrake(boolean brake) {
    io.setBrake(brake);
  }

  @AutoLogOutput(key = "PivotArm/Close")
  public boolean isVoltageClose(double setVoltage) {
    double voltageDifference = Math.abs(setVoltage - inputs.appliedVolts);
    return voltageDifference <= IntakeWristConstants.intakeWristPositionTolerance;
  }

  public void setVoltage(double motorVolts) {
    // limit the arm if its past the limit
    if (io.getAngle() > IntakeWristConstants.intakeWristMaxAngle && motorVolts > 0) {
      motorVolts = 0;
    } else if (io.getAngle() < IntakeWristConstants.intakeWristMinAngle && motorVolts < 0) {
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
    Logger.recordOutput("PivotArm/Setpoint", setpoint);
  }

  public void addPID(double setpointAdd) {
    this.setpoint += setpointAdd;
    this.setpoint =
        MathUtil.clamp(
            this.setpoint,
            IntakeWristConstants.intakeWristMinAngle,
            IntakeWristConstants.intakeWristMaxAngle);

    Logger.recordOutput("PivotArm/Setpoint", setpoint);
  }

  public boolean atSetpoint() {
    return Math.abs(io.getAngle() - setpoint) < IntakeWristConstants.intakeWristPositionTolerance
        && Math.abs(getVelocity()) < IntakeWristConstants.intakeWristVelocityTolerance;
  }

  public void setMechanism(MechanismLigament2d mechanism) {
    wristMechanism = mechanism;
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
    return wristMechanism.append(mechanism);
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
          Logger.recordOutput("PivotArmSpeakerAngle", setpointSupplier.getAsDouble());
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
}
