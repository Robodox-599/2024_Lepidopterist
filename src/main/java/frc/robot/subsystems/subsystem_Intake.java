// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.math.controller.PIDController;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.controls.VoltageOut;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.HardwareConfig;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SwerveConstants;

public class subsystem_Intake extends SubsystemBase {

  private TalonFX m_IntakeMotor;
  private TalonFX m_WristMotor;
  private DigitalInput m_BeamBreak1;
  private DutyCycleEncoder m_AbsEncoder;
  private HardwareConfig m_Settings;

  // private double m_IntakeEnc;
  private double m_IntakeSpeed;
  private double m_DesiredIntakePos;

  private double m_WristEnc; // where it at
  // private double m_WristExtend; //from in to out
  // private double m_WristRetract; //from out to in
  private double m_DesiredWristPos; //where wrist wants to go  
  private int m_WristSlot;

  /** Creates a new subsystem_Intake. */
  public subsystem_Intake() {
    m_IntakeMotor = new TalonFX(IntakeConstants.intakeMotorID, SwerveConstants.CANBus);
    m_WristMotor = new TalonFX(IntakeConstants.wristMotorID, SwerveConstants.CANBus);
    m_BeamBreak1 = new DigitalInput(IntakeConstants.breakerPort1);
    m_AbsEncoder = new DutyCycleEncoder(IntakeConstants.throughBoreEncID);
    m_Settings = new HardwareConfig();

    m_IntakeMotor.getConfigurator().apply(m_Settings.getMotorConfig(1));
    m_WristMotor.getConfigurator().apply(m_Settings.getMotorConfig(2));
    
    // m_IntakeEnc = 0.0;
    m_IntakeSpeed = 0.0;
    m_DesiredIntakePos = 0.0;

    m_WristEnc = 0.0; // thrubore enc val
    // m_WristExtend = 0.0; //from in to out
    // m_WristRetract = 0.0; //from out to in
    m_DesiredWristPos = 0.0; //where wrist wants to go
    m_WristSlot = 0;

    resetWristPos();
  }

  public void runIntake(){
    // VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    // m_intakeMotor.setControl(m_request.withVelocity(0).withFeedForward(0));
    m_IntakeMotor.set(m_IntakeSpeed);
  }

  public void stopIntake(){
    m_IntakeMotor.stopMotor();
  }

  public void resetWristPos(){
    m_AbsEncoder.reset();
  }

  public void setDesiredWristPos(double passedInPosition){
    m_DesiredWristPos = passedInPosition;
    m_WristSlot = passedInPosition == IntakeConstants.kWristExtendVal ? 
                                      IntakeConstants.wristExtendSlot : 
                                      IntakeConstants.wristRetractSlot;
  }

  public boolean isWristAtDesiredPosition(double passedInPosition){
    m_WristEnc = m_AbsEncoder.get()* IntakeConstants.wristGearRatio;
    return (Math.abs(m_WristEnc - passedInPosition) < IntakeConstants.kWristBufferZone);
  }

  public Command runIntakeCommand(){
    return Commands.sequence(new InstantCommand(() -> runIntake(), this)
                            .until(() -> {return !m_BeamBreak1.get();}), 
                            new InstantCommand(() -> stopIntake(), this));
  }

  public Command moveWristCommand(DoubleSupplier passedInPosition){
    return Commands.parallel(new InstantCommand(() -> setDesiredWristPos(passedInPosition.getAsDouble()), this))
                                                .until(() -> {return isWristAtDesiredPosition(m_DesiredIntakePos);});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Pos", m_IntakeMotor.get());
    SmartDashboard.putNumber("Wrist Thru Bore", m_AbsEncoder.get() * IntakeConstants.wristGearRatio);
    SmartDashboard.putNumber("Wrist Built In", m_WristMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Current Slot", m_WristSlot);
    SmartDashboard.putBoolean("At Desired Wrist Position", isWristAtDesiredPosition(m_DesiredWristPos));
    SmartDashboard.putNumber("Desired wrist pos", m_DesiredWristPos);

    // This method will be called once per scheduler run

    if(!isWristAtDesiredPosition(m_DesiredWristPos)){

      PositionVoltage m_request = new PositionVoltage(m_DesiredWristPos).withSlot(m_WristSlot);
      m_WristMotor.setControl(m_request.withPosition(m_DesiredWristPos));

    }
    else{
      m_WristMotor.stopMotor();
    }
  }
}
