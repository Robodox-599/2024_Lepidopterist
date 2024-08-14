package frc.robot.subsystems.intake.Rollers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

// testing live share :O

public class Rollers extends SubsystemBase {
  private final RollersIO io;
  RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

  public Rollers(RollersIO io) {
    this.io = io;
    SmartDashboard.putData(getName(), this);
  }

  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Rollers", inputs);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setBrake(boolean brake) {
    io.setBrake(brake);
  }

  public Command speedCommand(DoubleSupplier speed) {
    return new FunctionalCommand(
        () -> {}, () -> io.setSpeed(speed.getAsDouble()), (stop) -> io.stop(), () -> false, this);
  }

  public Command manualCommand(DoubleSupplier voltage) {
    return new FunctionalCommand(
        () -> {},
        () -> io.setVoltage(voltage.getAsDouble()),
        (stop) -> io.stop(),
        () -> false,
        this);
  }

  public Command manualCommand(double voltage) {
    return manualCommand(() -> voltage);
  }

  public Command stop() {
    return new FunctionalCommand(
        () -> {}, () -> io.setVoltage(0), (stop) -> io.stop(), () -> false, this);
  }
}
