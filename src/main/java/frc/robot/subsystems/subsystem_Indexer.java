// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.math.controller.PIDController;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.controls.VoltageOut;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.HardwareConfig;
import frc.robot.Constants.IndexerConstants;

public class subsystem_Indexer extends SubsystemBase {
  /** Creates a new subsystem_Indexer. */

  private TalonFX m_indexerMotor;
  private DigitalInput m_BeamBreak2;
  private HardwareConfig m_Settings;

  public subsystem_Indexer() {
    m_indexerMotor = new TalonFX(IndexerConstants.motorID);
    m_BeamBreak2 = new DigitalInput(IndexerConstants.breakerPort2);
    m_Settings = new HardwareConfig();
    m_indexerMotor.getConfigurator().apply(m_Settings.getMotorConfig(1));
    m_indexerMotor.setNeutralMode(NeutralModeValue.Coast);
    m_indexerMotor.setInverted(true);
  }

  public void runIndexer(){
    m_indexerMotor.set(IndexerConstants.kIndexerSpeed);
  }

  public void stopIndexer(){
    m_indexerMotor.stopMotor();
  }
  public Command stopIndexerCommand(){
    return new InstantCommand(() -> stopIndexer());
  }
  public Command runIndexerCommand(){
    return new InstantCommand(() -> runIndexer(), this);
  }
  public Command runIndexerUntilBeamBreak(){
    return Commands.sequence(new InstantCommand(() -> runIndexer(), this), 
                            new WaitUntilCommand(() -> {return !m_BeamBreak2.get();}), 
                            new InstantCommand(() -> stopIndexer(), this));
  }

  public Command runIndexerStartEnd(){
    return new StartEndCommand(() -> runIndexer(), () -> stopIndexer(), this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("beambreak2", !m_BeamBreak2.get());
  }
}
