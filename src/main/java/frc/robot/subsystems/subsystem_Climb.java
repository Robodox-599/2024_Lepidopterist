// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.SwerveConstants;
// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkLowLevel;

import frc.robot.HardwareConfig;
import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class subsystem_Climb extends SubsystemBase {
  /** Creates a new subsystem_Climb. */
  private TalonFX m_LeftClimbMotor;
  private TalonFX m_RightClimbMotor;
  private double m_DesiredPosLeft;
  private double m_DesiredPosRight;
  private double m_GyroRoll;
  
  // private SparkPIDController m_PidController_right;
  // private final RelativeEncoder m_encoder_right;
  private int m_State;
  private HardwareConfig m_Settings;

  public subsystem_Climb() {
    m_LeftClimbMotor = new TalonFX(ClimbConstants.leftMotorID, SwerveConstants.CANBus);
    m_RightClimbMotor = new TalonFX(ClimbConstants.rightMotorID, SwerveConstants.CANBus);
    m_Settings = new HardwareConfig();

    m_LeftClimbMotor.getConfigurator().apply(m_Settings.getMotorConfig(0));
    m_RightClimbMotor.getConfigurator().apply(m_Settings.getMotorConfig(0));

    m_DesiredPosLeft = 0.0;
    m_DesiredPosRight = 0.0;
    m_State = 0;
    m_GyroRoll = 0.0;
  }

  public void setClimbSetpoint(boolean extended, boolean retracted, double roll){
    m_GyroRoll = roll;
    double setpoint = 0.0;
    
    if(extended == retracted){
        setpoint = getLeftMotorPos();
    } else {
        setpoint = extended ? ClimbConstants.extended : ClimbConstants.retracted;
    }
    
    m_DesiredPosLeft = setpoint;
    m_DesiredPosRight = setpoint;
    m_State += 1;
  }

    public Command setSetpointCommand(BooleanSupplier extended, BooleanSupplier retracted, DoubleSupplier roll){
        return this.run(() -> setClimbSetpoint(extended.getAsBoolean(), 
                                              retracted.getAsBoolean(), 
                                              roll.getAsDouble()));
    }

    public double getLeftMotorPos(){
        return m_LeftClimbMotor.getPosition().getValueAsDouble();
    }

    public boolean isLevel(){
        return Math.abs(m_GyroRoll) < ClimbConstants.gyroThreshold;
    }

    public boolean isAtDesiredPos(){
        return Math.abs(m_LeftClimbMotor.getPosition().getValueAsDouble() - m_DesiredPosLeft) < ClimbConstants.setPointDeadband && 
            Math.abs(m_RightClimbMotor.getPosition().getValueAsDouble() - m_DesiredPosRight) < ClimbConstants.setPointDeadband;
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!isLevel() && m_State > 1){
        if(m_GyroRoll > 0){ //pull right or drop left
            m_DesiredPosLeft += Math.abs(m_GyroRoll) * ClimbConstants.errorGain;
            m_DesiredPosRight -= Math.abs(m_GyroRoll) * ClimbConstants.errorGain;
        }
        if(m_GyroRoll < 0){ //pull right or drop left
            m_DesiredPosLeft -= Math.abs(m_GyroRoll) * ClimbConstants.errorGain;
            m_DesiredPosRight += Math.abs(m_GyroRoll) * ClimbConstants.errorGain;
        }
    }

    if (!isAtDesiredPos()){
        PositionVoltage m_request = new PositionVoltage(m_DesiredPosLeft).withSlot(0);
        m_LeftClimbMotor.setControl(m_request.withPosition(m_DesiredPosLeft));
        PositionVoltage m_request2 = new PositionVoltage(m_DesiredPosRight).withSlot(0);
        m_RightClimbMotor.setControl(m_request2.withPosition(m_DesiredPosRight));
    } else {
        m_LeftClimbMotor.stopMotor();
        m_RightClimbMotor.stopMotor();
    }

    SmartDashboard.putBoolean("level", isLevel());
  }
}
