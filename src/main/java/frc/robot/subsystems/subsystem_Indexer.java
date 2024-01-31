// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IndexerConstants;


public class subsystem_Indexer extends SubsystemBase {

  private CANSparkMax m_indexerMotor;
  private double indexerSpeed;

  /** Creates a new subsystem_Indexer. */
  public subsystem_Indexer() {

    m_indexerMotor = new CANSparkMax(IndexerConstants.indexerMotorID, MotorType.kBrushless);
    m_indexerMotor.setIdleMode(IdleMode.kBrake);

    indexerSpeed = IndexerConstants.indexerSpeed;
  }

  public void runIndexer(){
    m_indexerMotor.set(indexerSpeed);
  }

  public void stopIndexer(){
    m_indexerMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Indexer Pos", m_indexerMotor.get());
    // This method will be called once per scheduler run
  }
}
