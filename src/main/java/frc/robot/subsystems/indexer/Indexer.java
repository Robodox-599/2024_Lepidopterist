package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.beamBreakDebounce;
import static frc.robot.subsystems.indexer.IndexerConstants.extraIndexerTime;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

// testing live share :O

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private Timer beamBreakTimer = new Timer();
  private DigitalInput m_BeamBreak2;
  public Indexer(IndexerIO io) {
    this.io = io;
    beamBreakTimer.start();  
    m_BeamBreak2 = new DigitalInput(IndexerConstants.beakBreak2Port);

    }

  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Indexer", inputs);
    Logger.recordOutput("Indexer/Updating", true);
    
    if (m_BeamBreak2.get()){
      beamBreakTimer.restart();
    }
    Logger.recordOutput("Indexer/BeamBrake triggered?", beamBreakTimer.get() >= IndexerConstants.beamBreakDebounce);
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
          io.setSpeed(0);
        });
  }

  public Command runIndexerBeamBreak(){
    return Commands.sequence(
      new InstantCommand(()-> setSpeed(0.4), this),
      new WaitUntilCommand(()-> (beamBreakTimer.get() >= beamBreakDebounce)),
      Commands.waitSeconds(extraIndexerTime),
      new InstantCommand(() -> stop(), this)
      );

  }

}
