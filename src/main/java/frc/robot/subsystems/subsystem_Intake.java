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

public class subsystem_Intake extends SubsystemBase {

  private TalonFX m_intakeMotor;
  private TalonFXConfiguration m_intakeConfig;

  private double intakeEnc;
  private double desiredIntakePos;
  private double intakeSpeed;

  /** Creates a new subsystem_Intake. */
  public subsystem_Intake() {
    m_intakeMotor = new TalonFX(IntakeConstants.intakeMotorID, SharedConstants.canbusID);
    m_intakeConfig = new TalonFXConfiguration();
    intakeSpeed = IntakeConstants.kIntakeSpeed;

    m_intakeConfig.Slot0.kP = IntakeConstants.intakeP;
    m_intakeConfig.Slot0.kI = IntakeConstants.intakeI;
    m_intakeConfig.Slot0.kD = IntakeConstants.intakeD;

    m_intakeMotor.getConfigurator().apply(m_intakeConfig);
  }

  public void runIntake(){

    // VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    //   m_intakeMotor.setControl(m_request.withVelocity(0).withFeedForward(0));

    m_intakeMotor.set(intakeSpeed);

  }

  public void stopIntake(){
    m_intakeMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Pos", m_intakeMotor.get());
  }
}
