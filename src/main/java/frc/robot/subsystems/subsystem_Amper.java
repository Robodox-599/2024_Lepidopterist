// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.Constants.ShooterWristConstants;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.revrobotics.CANSparkLowLevel;


public class subsystem_Amper extends SubsystemBase {
  /** Creates a new subsystem_Amper. */
  private TalonFX m_amper;
  private boolean m_amper_go;

  public subsystem_Amper() {
    m_amper = new TalonFX(Constants.ShooterFlywheelConstants.MotorConstants.amperID);
    m_amper_go = false;

  }
public void toggle_amper(boolean state){
    m_amper_go = state;
}

public Command amperCommand(boolean state){
    return this.runOnce(() -> toggle_amper(state));
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_amper_go == true){
      m_amper.setVoltage(Constants.ShooterFlywheelConstants.amperVoltage);
  }else{
      m_amper.setVoltage(0);
  }
  }
}
