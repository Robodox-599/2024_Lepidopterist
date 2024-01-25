// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class subsystem_Intake extends SubsystemBase {
  /** Creates a new subsystem_Intake. */

  private CANSparkMax m_intakeMotor;
  private double intakeSpeed;

  public subsystem_Intake() {

    intakeSpeed = IntakeConstants.kIntakeSpeed;
    m_intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);

    m_intakeMotor.setIdleMode(IdleMode.kBrake);
  }

  public void runIntake(){
    // m_intakePIDController.setReference(desiredIntakePos, ControlType.kPosition, desiredIntakeSlot);
    m_intakeMotor.set(intakeSpeed);
  }

  public void stopIntake(){
    m_intakeMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
