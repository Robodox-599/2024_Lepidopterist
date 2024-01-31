// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BreakerConstants;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class subsystem_Breakers extends SubsystemBase {
  /** Creates a new subsystem_Breakers. */
  DigitalInput m_breaker1;
  DigitalInput m_breaker2;


  public subsystem_Breakers() {
    m_breaker1 = new DigitalInput(BreakerConstants.breakerPort1);
    m_breaker2 = new DigitalInput(BreakerConstants.breakerPort2);

  }

  public boolean isFirstBeamBroken(){
    return(!(m_breaker1.get()));
  }

  public boolean isSecondBeamBroken(){
    return(!(m_breaker2.get()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("beam1 broken", isFirstBeamBroken());
    SmartDashboard.putBoolean("beam2 broken", isSecondBeamBroken());
  }
}
