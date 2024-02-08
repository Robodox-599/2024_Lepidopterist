// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;

import com.revrobotics.CANSparkLowLevel;


public class subsystem_Rod extends SubsystemBase {
/** Creates a new subsystem_Rod. */
private CANSparkMax m_motor;
// private TalonFXConfiguration m_motor_config;
private double m_desired_pos;
private SparkPIDController m_PidController;
private final RelativeEncoder m_encoder;

public subsystem_Rod() {
m_motor = new CANSparkMax(Constants.MotorID.motorID,CANSparkLowLevel.MotorType.kBrushless);
// m_motor_config = new TalonFXConfiguration();
m_encoder = m_motor.getEncoder();
m_PidController = m_motor.getPIDController();
m_PidController.setP(Constants.PIDConstants.extendP);

m_PidController.setI(Constants.PIDConstants.extendI);

m_PidController.setD(Constants.PIDConstants.extendD);

m_encoder.setPosition(0);
}


public void go (double setpoint){
m_desired_pos = setpoint;
}

public boolean is_at_desired(){
    return(Math.abs(m_encoder.getPosition() - m_desired_pos)<Constants.Setpoints.setPointDeadband);
    }
    
@Override
public void periodic() {
// This method will be called once per scheduler run
if (!is_at_desired()){

m_PidController.setReference(m_desired_pos,CANSparkMax.ControlType.kPosition, 0);

}else{
m_motor.stopMotor();
}
}
}



