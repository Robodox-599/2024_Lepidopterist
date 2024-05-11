// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IndexerConstants;
import frc.robot.HardwareConfig;

public class subsystem_Indexer extends SubsystemBase {
  /** Creates a new subsystem_Indexer. */
  private TalonFX m_indexerMotor;

  private DigitalInput m_BeamBreak2;
  private HardwareConfig m_Settings;

  private Timer m_beamBreakTimer = new Timer();

  public subsystem_Indexer() {
    m_indexerMotor = new TalonFX(IndexerConstants.motorID);
    m_BeamBreak2 = new DigitalInput(IndexerConstants.beakBreak2Port);
    m_Settings = new HardwareConfig();
    m_indexerMotor.getConfigurator().apply(m_Settings.getMotorConfig(2));
    m_indexerMotor.setNeutralMode(NeutralModeValue.Coast);
    m_indexerMotor.setInverted(true);
    m_beamBreakTimer.start();
    SmartDashboard.putBoolean("indexerBeamBreakTriggered", false);
  }

  public void runIndexer() {
    m_indexerMotor.set(IndexerConstants.kIndexerSpeed);
    // m_indexerMotor.setControl(new VelocityVoltage(IndexerConstants.intakeRPS));
  }

  public void runIndexerShooter() {
    m_indexerMotor.set(0.4);
    // m_indexerMotor.setControl(new VelocityVoltage(IndexerConstants.shootRPS));
  }

  public void stopIndexer() {
    m_indexerMotor.stopMotor();
  }

  public void runIndexerBackwards() {
    m_indexerMotor.set(IndexerConstants.kIndexerBackSpeed);
  }

  public void runIndexerSource() {
    m_indexerMotor.set(IndexerConstants.kIndexerSourceSpeed);
  }

  public Command stopIndexerCommand() {
    return new InstantCommand(() -> stopIndexer(), this);
  }

  public Command runIndexerUntilBeamBreak() {
    return Commands.sequence(
        new InstantCommand(() -> runIndexer(), this),
        new WaitUntilCommand(() -> (m_beamBreakTimer.get() >= IndexerConstants.beamBreakDebounce)),
        Commands.waitSeconds(IndexerConstants.extraIndexerTime),
        new InstantCommand(() -> stopIndexer(), this));
  }

  public Command runIndexerStartEnd() {
    return new StartEndCommand(() -> runIndexer(), () -> stopIndexer(), this);
  }

  public Command runIndexerShootStartEnd() {
    return new StartEndCommand(() -> runIndexerShooter(), () -> stopIndexer(), this);
  }

  public Command runIndexerBackwardsStartEnd() { // hear me out!
    return new StartEndCommand(() -> runIndexerBackwards(), () -> stopIndexer(), this);
  }

  public Command backfeedIndexerBeamBreak() {
    return Commands.sequence(
        new InstantCommand(() -> runIndexerBackwards(), this),
        new WaitUntilCommand(() -> (m_beamBreakTimer.get() <= IndexerConstants.beamBreakDebounce)),
        Commands.waitSeconds(IndexerConstants.extraIndexerTime),
        new InstantCommand(() -> stopIndexer(), this));
  }

  public Command sourceIndexer() {
    return Commands.sequence(
        new InstantCommand(() -> runIndexerSource(), this),
        new WaitUntilCommand(() -> (m_beamBreakTimer.get() >= IndexerConstants.beamBreakDebounce)),
        Commands.waitSeconds(IndexerConstants.extraSourceIndexerTime),
        new InstantCommand(() -> stopIndexer(), this));
  }

  @Override
  public void periodic() {
    if (m_BeamBreak2.get()) {
      m_beamBreakTimer.restart();
    }

    if (m_beamBreakTimer.get() >= IndexerConstants.beamBreakDebounce) {
      SmartDashboard.putBoolean("indexerBeamBreakTriggered", true);
    }

    SmartDashboard.putBoolean(
        "beambreak2", m_beamBreakTimer.get() >= IndexerConstants.beamBreakDebounce);
    SmartDashboard.putNumber("indexerSpeed", m_indexerMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("indexer pos", m_indexerMotor.getPosition().getValueAsDouble());
  }
}
