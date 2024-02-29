// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;


public class subsystem_Rod extends SubsystemBase {
/** Creates a new subsystem_Rod. */
private TalonFX m_motor;
private TalonFXConfiguration m_motor_config;
private double m_desired_pos;
public subsystem_Rod() {
m_motor = new TalonFX(Constants.MotorID.motorID, Constants.MotorID.CANbusID);
m_motor_config = new TalonFXConfiguration();
m_motor_config.Slot0.kP = Constants.PIDConstants.extendP;
m_motor_config.Slot0.kI = Constants.PIDConstants.extendI;
m_motor_config.Slot0.kD = Constants.PIDConstants.extendD;




m_motor.getConfigurator().apply(m_motor_config);
}


public void go (double setpoint){
m_desired_pos = setpoint;
}

public boolean is_at_desired(){
return(Math.abs(m_motor.getPosition().getValueAsDouble() - m_desired_pos)<Constants.Setpoints.setPointDeadband);
}

@Override
public void periodic() {
// This method will be called once per scheduler run
if (!is_at_desired()){

PositionVoltage m_request = new PositionVoltage(m_desired_pos).withSlot(0);
m_motor.setControl(m_request.withPosition(m_desired_pos));
}else{
m_motor.stopMotor();
}
}
}



