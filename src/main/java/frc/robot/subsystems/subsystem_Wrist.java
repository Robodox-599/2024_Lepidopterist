// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SharedConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.IndexerConstants;

import edu.wpi.first.wpilibj.DutyCycleEncoder;


public class subsystem_Wrist extends SubsystemBase {

  private TalonFX m_wristMotor;
  private DutyCycleEncoder thruEncoder;
  private TalonFXConfiguration m_wristConfig;

  private double wristPosition; // where it at
  private double wristEnc; // thrubore enc val

  private double wristExtend; //from in to out
  private double wristRetract; //from out to in

  private double desiredWristPos; //where wrist wants to go
  private int desiredWristSlot; //PID slot, controls speed respectively to direction

  //Have to transfer units from tick to radians at some point
  
  public subsystem_Wrist() {

    m_wristMotor = new TalonFX(WristConstants.wristMotorID, SharedConstants.canbusID);
    thruEncoder = new DutyCycleEncoder(WristConstants.absEncoderChannel);
    m_wristConfig = new TalonFXConfiguration();
  
    //wrist extend PID SLOT 0
    m_wristConfig.Slot0.kP = WristConstants.kWristExtendP;
    m_wristConfig.Slot0.kI = WristConstants.kWristExtendI;
    m_wristConfig.Slot0.kD = WristConstants.kWristExtendD;
    // SmartDashboard.putNumber("WristExtend P", WristConstants.kWristExtendP);
    // SmartDashboard.putNumber("WristExtend I", WristConstants.kWristExtendI);
    // SmartDashboard.putNumber("WristExtend D",WristConstants.kWristExtendD);

    //wrist retract PID SLOT 1
    m_wristConfig.Slot1.kP = WristConstants.kWristRetractP;
    m_wristConfig.Slot1.kI = WristConstants.kWristRetractI;
    m_wristConfig.Slot1.kD = WristConstants.kWristRetractD;
    // SmartDashboard.putNumber("WristRetract P", WristConstants.kWristRetractP);
    // SmartDashboard.putNumber("WristRetract I", WristConstants.kWristRetractI);
    // SmartDashboard.putNumber("WristRetract D", WristConstants.kWristRetractD);
   
    //wristPosition = m_wristMotor.getPosition().getValueAsDouble();


    // wana set idle mode here

    m_wristMotor.set(0);

    m_wristMotor.getConfigurator().apply(m_wristConfig);
  }

  public void resetWristPos(){
    thruEncoder.reset();
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
    //

  }

  public boolean isWristAtDesiredPosition(double passedInPosition){
    wristEnc = thruEncoder.get()* WristConstants.gearRatio;
    return (Math.abs(wristEnc - passedInPosition) < WristConstants.kWristBufferZone);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Wrist Thru Bore", thruEncoder.get()*WristConstants.gearRatio);
    SmartDashboard.putNumber("Wrist Built In", m_wristMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Current Slot", desiredWristSlot);
    SmartDashboard.putBoolean("At Desired Wrist Position", isWristAtDesiredPosition(desiredWristPos));
    SmartDashboard.putNumber("Desired wrist pos", desiredWristPos);

    // This method will be called once per scheduler run

    if(!isWristAtDesiredPosition(desiredWristPos)){

      PositionVoltage m_request = new PositionVoltage(desiredWristPos).withSlot(desiredWristSlot);
      m_wristMotor.setControl(m_request.withPosition(desiredWristPos));

    }
    else{
      m_wristMotor.stopMotor();
    }

    

  //private DutyCycleEncoder m_wristThroughboreEncoder;
  //private SparkMaxAbsoluteEncoder m_wristThroughBoreEncoder;
  //private Encoder m_wristThroughboreEncoder;

  // For Review
  // public boolean wristThreshold(double threshold){
  //   // return( desiredWristPosition >= wristThroughboreEncoderValue ? wristThroughboreEncoderValue > threshold : wristThroughboreEncoderValue < threshold);
  //   if(desiredWristPosition >= wristThroughboreEncoderValue){
  //     return wristThroughboreEncoderValue > Constants.WristConstants.wristThresholdVal;
  // } else{
  //     return wristThroughboreEncoderValue < Constants.WristConstants.wristThresholdVal;
  // }
  // }
  }
}
