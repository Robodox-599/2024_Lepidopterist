// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.LEDConstants.LEDAnimation;
import frc.robot.Constants.ShooterConstants.FlywheelSetpoints;
import frc.robot.Constants.ShooterConstants.WristSepoints;
import frc.robot.subsystems.subsystem_Shooter;
import frc.robot.subsystems.subsystem_LED;
import frc.robot.subsystems.subsystem_DriveTrain;

public class cGroup_Shooter {
  /** Creates a new command_cGroup_shooter. */
  public static Command armSpeakerCommand(subsystem_Shooter m_Shooter, subsystem_LED m_LED, subsystem_DriveTrain m_DriveTrain){
    return new command_ToWristAndSpeed(m_Shooter, 
    () -> WristSepoints.testSpeakerWrist, 
    () -> FlywheelSetpoints.SpeakerSpeed);
  }
  
  public static Command scoreAmp (subsystem_Shooter m_Shooter, subsystem_LED m_LED){
    return Commands.parallel(new command_ToWristAndSpeed(m_Shooter, 
                            () -> WristSepoints.ampWrist, 
                            () -> FlywheelSetpoints.StowSpeed), 
                            m_LED.setAnimationCommand(LEDAnimation.Amping, () -> 2.0));
  }

  public static Command sourceIntake(subsystem_Shooter m_Shooter, subsystem_LED m_LED){
    return Commands.parallel(new command_ToWristAndSpeed(m_Shooter, 
                            () -> WristSepoints.sourceWrist, 
                            () -> FlywheelSetpoints.SourceSpeed), 
                            m_LED.setAnimationCommand(LEDAnimation.Intook, () -> 2.0));
  }
  
    //TODO: Add rumble once note is intook
  
  
    // Called when the command is initially scheduled.
}
