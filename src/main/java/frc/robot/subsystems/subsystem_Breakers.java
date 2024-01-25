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
  DigitalInput m_breaker;


  public subsystem_Breakers() {
    m_breaker = new DigitalInput(BreakerConstants.breakerPort);

  }

  public boolean isBeamBroken(){
    return(!(m_breaker.get()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("beam broken", !isBeamBroken());
  }
}
