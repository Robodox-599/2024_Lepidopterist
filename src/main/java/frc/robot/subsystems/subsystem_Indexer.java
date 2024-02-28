// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.math.controller.PIDController;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.controls.VoltageOut;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.HardwareConfig;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.IndexerConstants;

public class subsystem_Indexer extends SubsystemBase {
  /** Creates a new subsystem_Indexer. */

  private TalonFX m_indexerMotor;
  private DigitalInput m_BeamBreak2;
  private HardwareConfig m_Settings;

  public subsystem_Indexer() {
    m_indexerMotor = new TalonFX(IndexerConstants.indexerMotorID, SwerveConstants.CANBus);
    m_BeamBreak2 = new DigitalInput(IndexerConstants.breakerPort2);
    m_Settings = new HardwareConfig();

    m_indexerMotor.getConfigurator().apply(m_Settings.getMotorConfig(0));
  }

  public void runIndexer(){
    // VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    //   m_indexerMotor.setControl(m_request.withVelocity(0).withFeedForward(0));
    m_indexerMotor.set(IndexerConstants.kIndexerSpeed);

  }

  public void stopIndexer(){
    m_indexerMotor.stopMotor();
  }

  public Command runIndexerCommand(){
    return Commands.sequence(new InstantCommand(() -> runIndexer(), this)
                            .until(() -> {return !m_BeamBreak2.get();}), 
                            new InstantCommand(() -> stopIndexer(), this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
