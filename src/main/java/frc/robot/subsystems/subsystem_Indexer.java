// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SharedConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.IndexerConstants;

public class subsystem_Indexer extends SubsystemBase {
  /** Creates a new subsystem_Indexer. */

  private TalonFX m_indexerMotor;
  private TalonFXConfiguration m_indexerConfig;
  private double indexerSpeed;

 

  public subsystem_Indexer() {
    m_indexerMotor = new TalonFX(IndexerConstants.indexerMotorID);
    m_indexerConfig = new TalonFXConfiguration();
    indexerSpeed = IndexerConstants.kIndexerSpeed;

     m_indexerConfig.Slot0.kP = IndexerConstants.indexerP;
    m_indexerConfig.Slot0.kI = IndexerConstants.indexerI;
    m_indexerConfig.Slot0.kD = IndexerConstants.indexerD;

    m_indexerMotor.getConfigurator().apply(m_indexerConfig);

  }

  public void runIndexer(){
    // VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    //   m_indexerMotor.setControl(m_request.withVelocity(0).withFeedForward(0));
    m_indexerMotor.set(indexerSpeed);
  }

  public void stopIndexer(){
    m_indexerMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
