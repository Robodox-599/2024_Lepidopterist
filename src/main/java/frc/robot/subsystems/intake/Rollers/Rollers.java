package frc.robot.subsystems.intake.rollers;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.intake.IntakeConstants;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

// testing live share :O

public class Rollers extends SubsystemBase {
  private final RollersIO io;
  RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();
  private Timer beamBreakTimer = new Timer();
  private DigitalInput beamBreak1;

  public Rollers(RollersIO io) {
    this.io = io;
    SmartDashboard.putData(getName(), this);
    beamBreakTimer.start();
    beamBreak1 = new DigitalInput(IntakeConstants.beamBreak1Port);
  }

  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Rollers", inputs);
    if(beamBreak1.get()){
      beamBreakTimer.restart();
    }

  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setBrake(boolean brake) {
    io.setBrake(brake);
  }

  public Command setSpeed(double speed) {
    return Commands.run(
        () -> {
          io.setSpeed(speed);
        });
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
  public boolean getBeamBreak(){
    return(beamBreakTimer.get() >= IndexerConstants.beamBreakDebounce)
  }
}
