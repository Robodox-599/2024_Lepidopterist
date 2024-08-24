package frc.robot.subsystems.indexer;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.indexer.IndexerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    Logger.recordOutput("Indexer/Updating", true);
    getVoltage();
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

  public Command setSpeed(double speed) {
    return Commands.run(
        () -> {
          io.setSpeed(speed);
        });
  }

  public Command stop() {
    return Commands.run(
        () -> {
          io.setVoltage(0);
        });
  }
}
