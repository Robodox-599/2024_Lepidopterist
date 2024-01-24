// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.Constants.WristConstants;

public class subsystem_Wrist extends SubsystemBase {
  private CANSparkMax m_wristMotor;
  private double desiredWristPos;
  private SparkPIDController m_WristPIDController;
  private RelativeEncoder m_wristEncoder;
  private int desiredWristSlot;

  /** Creates a new subsystem_Wrist. */
  public subsystem_Wrist() {
      m_wristMotor = new CANSparkMax(WristConstants.wristMotorID, MotorType.kBrushless);
      m_wristEncoder = m_wristMotor.getEncoder();
      m_WristPIDController = m_wristMotor.getPIDController();

      m_WristPIDController.setP(WristConstants.kWristExtendP, WristConstants.wristExtendSlot);
      m_WristPIDController.setI(WristConstants.kWristExtendI, WristConstants.wristExtendSlot);
      m_WristPIDController.setD(WristConstants.kWristExtendD, WristConstants.wristExtendSlot);

      m_WristPIDController.setP(WristConstants.kWristRetractP, WristConstants.wristRetractSlot);
      m_WristPIDController.setI(WristConstants.kWristRetractI, WristConstants.wristRetractSlot);
      m_WristPIDController.setD(WristConstants.kWristRetractD, WristConstants.wristRetractSlot);

      m_wristMotor.setIdleMode(IdleMode.kBrake);


  }

  public void setDesiredWristPos(double passedInPosition){

    desiredWristPos = passedInPosition;

    // Makes sure current position is the correct slot (future hannah trust)
    if(passedInPosition == WristConstants.kWristExtendVal){
      desiredWristSlot = WristConstants.wristExtendSlot;
    }
    else{
      desiredWristSlot = WristConstants.wristRetractSlot;
    }

  }

  public boolean isWristAtDesiredPosition(double passedInPosition){
    return (Math.abs(passedInPosition - m_wristEncoder.getPosition()) < WristConstants.kWristBufferZone);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // SmartDashboard.putNumber("Wrist Built-in Encoder", m_wristEncoder.getPosition());
    // SmartDashboard.putNumber("Current Slot", desiredWristSlot);
    // SmartDashboard.putBoolean("At Desired Wrist Position", isWristAtDesiredPosition(desiredWristPos));

    if(!isWristAtDesiredPosition(desiredWristPos)){
      m_WristPIDController.setReference(desiredWristPos, ControlType.kPosition, desiredWristSlot);
    }
    else{
      m_wristMotor.stopMotor();
    }

  }
}
