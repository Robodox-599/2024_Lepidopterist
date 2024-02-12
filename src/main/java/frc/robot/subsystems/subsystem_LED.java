// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.LEDConstants.LEDState;

import java.util.Map;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class subsystem_LED extends SubsystemBase {
  /** Creates a new subsystem_LED. */
  CANdle m_candle;
  LEDState m_LEDState = LEDState.OFF;

  public subsystem_LED() {
    m_candle = new CANdle(Constants.LEDConstants.CANdle_ID);
    m_candle.configLEDType(LEDStripType.GRB);
    m_candle.configBrightnessScalar(0.5);
    // m_candle.configLOSBehavior(true); uncomment if wanted, but not sure if needed
    SmartDashboard.putBoolean("LED Color", false);
    Shuffleboard.getTab("SmartDashboard").add("LED Color", true)
    .withProperties(Map.of("Color when true", "#FFBF00", 
    "Color when false", (RobotConstants.robotColor == Alliance.Red)?"red":"blue")).getEntry();
  }

  public LEDState getLEDState(){
    return m_LEDState;
  }

  public void toggleAmpCoop(){
    if (m_LEDState == LEDState.AMP){
      SmartDashboard.putBoolean("LED Color", true);
      m_LEDState = LEDState.COOP;
    } else {
      m_LEDState = LEDState.AMP;
      SmartDashboard.putBoolean("LED Color", false);
    }
  }

  public Command toggleAmpCoopCommand(){
    return this.runOnce(() -> toggleAmpCoop());
  }

  @Override
  public void periodic() {
    switch(m_LEDState){
        case AMP:
          if (RobotConstants.robotColor == Alliance.Red){
            m_candle.setLEDs(255, 0, 0);
          } else {
            m_candle.setLEDs(0, 0, 255);
          }
          break;
        case COOP:
          m_candle.setLEDs(255, 191, 0);
          break;
        case OFF:
          m_candle.setLEDs(0, 0, 0);
          break;
      }
    // This method will be called once per scheduler run
  }
}
