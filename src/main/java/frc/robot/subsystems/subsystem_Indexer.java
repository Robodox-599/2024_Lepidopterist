// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;


public class subsystem_Indexer extends SubsystemBase {
/** Creates a new subsystem_Indexer. */
private TalonFX m_motor;
private TalonFXConfiguration m_motor_config;
private double m_desired_pos;
DigitalInput m_breaker1;
DigitalInput m_breaker2;
private int state;

public subsystem_Indexer() {
m_motor = new TalonFX(Constants.IndexerConstants.motorID, Constants.IndexerConstants.canbus);
m_motor_config = new TalonFXConfiguration();
m_motor_config.Slot0.kP = Constants.IndexerConstants.IndexP;
m_motor_config.Slot0.kI = Constants.IndexerConstants.IndexI;
m_motor_config.Slot0.kD = Constants.IndexerConstants.IndexD;

m_desired_pos=10;

state =-2;
m_breaker1 = new DigitalInput(Constants.IndexerConstants.breakerPort1);
m_breaker2 = new DigitalInput(Constants.IndexerConstants.breakerPort2);

m_motor.getConfigurator().apply(m_motor_config);
}

public boolean isFirstBeamBroken(){
    return(!(m_breaker1.get()));
  }

  public boolean isSecondBeamBroken(){
    return(!(m_breaker2.get()));
  }
public void go (double setpoint){
m_desired_pos = setpoint;
state +=2;
}

public boolean is_at_desired(){
return(Math.abs(m_motor.getPosition().getValueAsDouble() - m_desired_pos)<Constants.IndexerConstants.setPointDeadband);
}

@Override
public void periodic() {
SmartDashboard.putNumber("desired", m_desired_pos);
SmartDashboard.putBoolean("beam1 broken", isFirstBeamBroken());
SmartDashboard.putNumber("state", state);
SmartDashboard.putBoolean("beam2 broken", isSecondBeamBroken());
if (isSecondBeamBroken() && (state%2 ==0)){
        Timer.delay(0.125);
        m_desired_pos=15;
        m_motor.set(0);
        state+=1;
    }

// This method will be called once per scheduler run
if (m_desired_pos== Constants.Setpoints.extended){
    m_motor.set(Constants.Setpoints.PoutHigh);

    
}else if (m_desired_pos == Constants.Setpoints.stow){
    m_motor.set(-Constants.Setpoints.PoutHigh);
// this is the right way
    
}
else if (m_desired_pos == 15){
    m_motor.set(0.0);
    state =1;
    
}
else if (m_desired_pos == 20){
    // m_motor.set(Constants.Setpoints.PoutLow);
    state=0;

    
}


}}