package frc.robot.subsystems.indexer;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.indexer.IndexerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// testing live share :O

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO io) {
    this.io = io;
    switch (robotType) {
      case REALBOT:
        io.configurePID(realkP, realkI, realkD);
        break;
      case REPLAYBOT:
        io.configurePID(simkP, simkI, simkD);
        break;
      case SIMBOT:
        io.configurePID(simkP, simkI, simkD);
        break;
    }
  }

  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Indexer", inputs);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setBrake(boolean brake) {
    io.setBrake(brake);
  }

  @AutoLogOutput(key = "Indexer/IndexerAppliedVoltage")
  public double getVoltage() {
    return inputs.appliedVoltage;
  }

  public Command speedCommand(DoubleSupplier speed) {
    return new FunctionalCommand(
        () -> io.setSpeed(speed.getAsDouble()),
        () -> io.setSpeed(speed.getAsDouble()),
        (interrupted) -> {
          if (interrupted) {
            io.stop();
          }
        },
        () -> false,
        this);
  }

  public Command manualCommand(DoubleSupplier voltage) {
    return new FunctionalCommand(
        () -> io.setVoltage(voltage.getAsDouble()),
        () -> io.setVoltage(voltage.getAsDouble()),
        (interrupted) -> {
          if (interrupted) {
            io.stop();
          }
        },
        () -> false,
        this);
  }

  public Command manualCommand(double voltage) {
    return manualCommand(() -> voltage);
  }

  public Command stop() {
    return new FunctionalCommand(
        () -> io.setVoltage(0),
        () -> io.setVoltage(0),
        (interrupted) -> {
          if (interrupted) {
            io.stop();
          }
        },
        () -> false,
        this);
  }
}
