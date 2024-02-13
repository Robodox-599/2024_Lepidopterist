// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class Subsytem_climb extends SubsystemBase {
  /** Creates a new Subsytem_climb. */
    private TalonFX m_climbMotor;
    private TalonFXConfiguration m_climbConfig;
    // private double climbEncoder; 
    // private double increment1; //move to to constant //represents distance from base to top
    //derive increment as:
    // (distance to move (in meters)/circumference of driving gear)/gear ratio_(motor to driving gear given in turns of driving gear/turns of motor)
    //increment is in terms of rotations (as units)
    private double climbTop;
    private double climbBottom;
    private double desired_Position;
    // private double step;
  public Subsytem_climb() {
    m_climbMotor = new TalonFX(Constants.MotorID.climbMotorID, Constants.MotorID.CANbusID);
    m_climbConfig = new TalonFXConfiguration();
    m_climbConfig.Slot0.kP = Constants.PIDConstants.climbP;
    m_climbConfig.Slot0.kI = Constants.PIDConstants.climbI;
    m_climbConfig.Slot0.kD = Constants.PIDConstants.climbD;
    m_climbConfig.Slot0.kS = Constants.PIDConstants.climbS;
    m_climbConfig.Slot0.kV = Constants.PIDConstants.climbV;

    desired_Position = climbBottom;
    // increment1 = 100;//calculate using gear ratios and whatnot distance from base to top with margin
    // step = 5;
    climbTop = m_climbMotor.getPosition().getValueAsDouble() + Constants.Setpoints.increment1;
    climbBottom = m_climbMotor.getPosition().getValueAsDouble();
    m_climbMotor.setPosition(0);
    m_climbMotor.getConfigurator().apply(m_climbConfig);
    
  }

  public void goClimb(double state){
   // m_climbMotor.getPosition().getValueAsDouble();
   if (state == 1 && (desired_Position+Constants.Setpoints.step< climbTop) ) {
    desired_Position += Constants.Setpoints.step;
   } 
    else if (state == 2 && (desired_Position-Constants.Setpoints.step >climbBottom)){
      desired_Position -= Constants.Setpoints.step;
    } else if(state == 0){
      m_climbMotor.set(0);

      desired_Position = m_climbMotor.getPosition().getValueAsDouble();// delete if necessary
      VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
      m_climbMotor.setControl(m_request.withVelocity(0).withFeedForward(0)); //Constants.PIDConstants.kinetic_friction
    }
    SmartDashboard.putNumber("state", state);
  }
  @Override
  public void periodic() {
SmartDashboard.putNumber("desired", desired_Position);
SmartDashboard.putNumber("climb top", climbTop);
SmartDashboard.putNumber("climb bottom", climbBottom);
SmartDashboard.putNumber("encoder pos", m_climbMotor.getPosition().getValueAsDouble());

    if ((Math.abs(m_climbMotor.getPosition().getValueAsDouble() - desired_Position) > Constants.Setpoints.setPointDeadband)){

    PositionVoltage m_request = new PositionVoltage(desired_Position).withSlot(0);
    m_climbMotor.setControl(m_request.withPosition(desired_Position));


    }else{
    m_climbMotor.stopMotor();
    }
    // This method will be called once per scheduler run
  }
}
