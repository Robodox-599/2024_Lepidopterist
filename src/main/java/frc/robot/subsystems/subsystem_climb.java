// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;

import com.revrobotics.CANSparkLowLevel;


public class subsystem_climb extends SubsystemBase {
/** Creates a new subsystem_climb. */
private TalonFX m_motorLeft;
private TalonFX m_motorRight;
// private TalonFXConfiguration m_motorLeft_config;
private double m_desired_pos_left;
private double m_desired_pos_right;
private TalonFXConfiguration m_LeftConfiguration;
private TalonFXConfiguration m_RightConfiguration;

// private SparkPIDController m_PidController_right;
// private final RelativeEncoder m_encoder_right;
private int m_state;

private Pigeon2 m_gyro;

public subsystem_climb() {

m_gyro = new Pigeon2(Constants.MotorID.gyroID, "rio");// add "DriveCANivore") if needed
m_gyro.getConfigurator().apply(new Pigeon2Configuration());
    zeroGyro();


m_LeftConfiguration = new TalonFXConfiguration();

m_RightConfiguration = new TalonFXConfiguration();
m_motorLeft = new TalonFX(Constants.MotorID.motorID_Left,"rio");
// m_motorLeft_config = new TalonFXConfiguration();
// m_encoder_left = m_motorLeft.getEncoder();
// m_PidController_left = m_motorLeft.getPIDController();

m_LeftConfiguration.Slot0.kP=(Constants.PIDConstants.extendP);
m_LeftConfiguration.Slot0.kI=(Constants.PIDConstants.extendI);
m_LeftConfiguration.Slot0.kD=(Constants.PIDConstants.extendD);


m_LeftConfiguration.Slot1.kP=(Constants.PIDConstants.balanceP);
m_LeftConfiguration.Slot1.kI=(Constants.PIDConstants.balanceI);
m_LeftConfiguration.Slot1.kD=(Constants.PIDConstants.balanceD);


m_motorRight = new TalonFX(Constants.MotorID.motorID_Right);
// m_motorRight_config = new TalonFXConfiguration();
// m_encoder_right = m_motorRight.getEncoder();
// m_PidController_right = m_motorRight.getPIDController();
m_RightConfiguration.Slot0.kP=(Constants.PIDConstants.extendP);
m_RightConfiguration.Slot0.kI=(Constants.PIDConstants.extendI);
m_RightConfiguration.Slot0.kD=(Constants.PIDConstants.extendD);

m_RightConfiguration.Slot1.kP=(Constants.PIDConstants.balanceP);
m_RightConfiguration.Slot1.kI=(Constants.PIDConstants.balanceI);
m_RightConfiguration.Slot1.kD=(Constants.PIDConstants.balanceD);

m_motorLeft.getConfigurator().apply(m_LeftConfiguration);
m_motorRight.getConfigurator().apply(m_RightConfiguration);

// m_encoder_left.setPosition(0);
// m_desired_pos_left = Constants.Setpoints.stow;
// m_desired_pos_right = Constants.Setpoints.stow;

m_state = 0;
}


public void go (double setpoint){
m_desired_pos_left = setpoint;
m_desired_pos_right = setpoint;
m_state += 1;
}

public void zeroGyro(){
    m_gyro.setYaw(0.0);
}
public double getPose(){
    return m_motorLeft.getPosition().getValueAsDouble();
}

public boolean is_at_desired(){
    return((Math.abs(m_motorLeft.getPosition().getValueAsDouble() - m_desired_pos_left)<Constants.Setpoints.setPointDeadband)
    &&(Math.abs(m_motorRight.getPosition().getValueAsDouble() - m_desired_pos_right)<Constants.Setpoints.setPointDeadband));
    }
    
public boolean is_level(){
    return(Math.abs(m_gyro.getRoll().getValueAsDouble())<Constants.GyroConstants.gyroThreshold);
}
@Override
public void periodic() {
// This method will be called once per scheduler 
//(Math.abs(m_desired_pos_right-Constants.Setpoints.retracted)<Constants.Setpoints.balanceLimit)

SmartDashboard.putBoolean("level", is_level());

//m_state  = 1 when climb extends, =2 when retracting. >1 to account for repress 
if (!is_level() && m_state > 1){
    if(m_gyro.getRoll().getValueAsDouble() > 0){ //pull right or drop left
        m_desired_pos_left+=(Math.abs(m_gyro.getRoll().getValueAsDouble())*Constants.GyroConstants.errorGain);
        m_desired_pos_right-=(Math.abs(m_gyro.getRoll().getValueAsDouble())*Constants.GyroConstants.errorGain);

    }
    if(m_gyro.getRoll().getValueAsDouble() < 0){ //pull right or drop left
        m_desired_pos_left-=(Math.abs(m_gyro.getRoll().getValueAsDouble())*Constants.GyroConstants.errorGain);
        m_desired_pos_right+=(Math.abs(m_gyro.getRoll().getValueAsDouble())*Constants.GyroConstants.errorGain);

    }

}

if (!is_at_desired()){
    PositionVoltage m_request = new PositionVoltage(m_desired_pos_left).withSlot(0);
    m_motorLeft.setControl(m_request.withPosition(m_desired_pos_left));
    PositionVoltage m_request2 = new PositionVoltage(m_desired_pos_right).withSlot(0);
    m_motorRight.setControl(m_request2.withPosition(m_desired_pos_right));
}else{
m_motorLeft.stopMotor();
m_motorRight.stopMotor();
}
}
}



