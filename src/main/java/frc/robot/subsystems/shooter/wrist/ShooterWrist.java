package frc.robot.subsystems.shooter.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterWrist extends SubsystemBase {
  private final ShooterWristIOInputsAutoLogged inputs = new ShooterWristIOInputsAutoLogged();

  private double setpoint = 0;

  private final ShooterWristIO io;
  private boolean isHomed = false;
  // Create a Mechanism2d visualization of the arm
  private MechanismLigament2d armMechanism = getArmMechanism();

  public ShooterWrist(ShooterWristIO io) {
    this.io = io;

    SmartDashboard.putData(getName(), this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ShooterWrist", inputs);

    armMechanism.setAngle(Units.radiansToDegrees(inputs.angleRads));

    // Log Inputs
    Logger.processInputs("ShooterWrist", inputs);
  }

  public void setBrake(boolean brake) {
    io.setBrake(brake);
  }

  @AutoLogOutput(key = "ShooterWrist/Close")
  public boolean isVoltageClose(double setVoltage) {
    double voltageDifference = Math.abs(setVoltage - inputs.appliedVolts);
    return voltageDifference <= ShooterWristConstants.shooterWristPIDTolerance;
  }

  public void setVoltage(double motorVolts) {
    // limit the wrist if its past the limit
    if (io.getAngle() > ShooterWristConstants.shooterWristMaxAngle && motorVolts > 0) {
      motorVolts = 0;
    } else if (io.getAngle() < ShooterWristConstants.shooterWristMinAngle && motorVolts < 0) {
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
    Logger.recordOutput("ShooterWrist/Setpoint", setpoint);
  }

  public void addPID(double setpointAdd) {
    this.setpoint += setpointAdd;
    this.setpoint = MathUtil.clamp(this.setpoint, 0, 14);

    Logger.recordOutput("ShooterWrist/Setpoint", setpoint);
  }

  public boolean atSetpoint() {
    return Math.abs(io.getAngle() - setpoint) < ShooterWristConstants.shooterWristPIDTolerance
        && Math.abs(getVelocity()) < ShooterWristConstants.shooterWristVelocityTolerance;
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
    return new MechanismLigament2d("ShooterWrist", 0.4, 0, 5, new Color8Bit(Color.kAqua));
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
          Logger.recordOutput("ShooterWrist/SpeakerAngle", setpointSupplier.getAsDouble());
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

  public Command zero() {
    return new InstantCommand(() -> io.zeroPosition());
  }

  public Command homeCommand() {
    return new WaitUntilCommand(() -> home());
  }

  public boolean home() {
    if (io.homeWrist(true)) {
      isHomed = true;
      return true;
    }
    return false;
  }

  // public Command incrementShooterUp() {
  //   return Commands.run(io.incrementShooterUp();
  // }

  // public Command incrementShooterDown() {
  //   return Commands.run(io.incrementShooterUp());
  // }
}
