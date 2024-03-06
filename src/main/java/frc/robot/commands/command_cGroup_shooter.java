// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.subsystem_ShooterWheels;
import frc.robot.subsystems.subsystem_ShooterWrist;
import frc.robot.subsystems.subsystem_LED;
import frc.robot.subsystems.subsystem_DriveTrain;
public class command_cGroup_shooter {
  /** Creates a new command_cGroup_shooter. */
  // private String state_machine = "";
//  public static Command armAmpCommand(subsystem_ShooterWheels m_flywheels, subsystem_ShooterWrist m_wrist, subsystem_LED m_LED, subsystem_DriveTrain m_DriveTrain) {
//   return Commands.parallel(new command_ToFlywheelSpeed(m_flywheels, () ->                                                     
//                             Constants.ShooterFlywheelConstants.AmpSpeed),
//                              new command_ToSetWrist(m_wrist, ()->Constants.ShooterWristConstants.ampWrist)
//                                       ) 
//                                     ;
//  }
 public static Command shoot(subsystem_ShooterWheels m_flywheels, subsystem_ShooterWrist m_wrist, subsystem_LED m_LED, subsystem_DriveTrain m_DriveTrain){
 return m_LED.set_stateCommand("shoot") ;
 }
//  public static Command testShot (subsystem_ShooterWheels m_flywheels, subsystem_ShooterWrist m_wrist, subsystem_LED m_LED, subsystem_DriveTrain m_DriveTrain){
//   return Commands.parallel(m_LED.se)
//  }
 public static Command armSpeakerCommand(subsystem_ShooterWheels m_flywheels, subsystem_ShooterWrist m_wrist, subsystem_LED m_LED, subsystem_DriveTrain m_drivetrain){
  // state_machine = "armSpeaker";
// return Commands.parallel(new command_AutoSpeaker(m_wrist, m_drivetrain,m_flywheels));
return Commands.parallel(new command_ToSetWrist(m_wrist, ()-> Constants.ShooterWristConstants.testSpeakerWrist), new command_ToFlywheelSpeed(m_flywheels, ()->Constants.ShooterFlywheelConstants.SpeakerSpeed));
 }
 
public static Command scoreAmp (subsystem_ShooterWheels m_flywheels,subsystem_ShooterWrist m_wrist, subsystem_LED m_led){
return Commands.parallel(new command_ToSetWrist(m_wrist, ()->Constants.ShooterWristConstants.ampWrist),
new command_ToFlywheelSpeed(m_flywheels, ()-> Constants.ShooterFlywheelConstants.AmpSpeed), m_led.set_stateCommand("shoot"));
}

public static Command sourceIntake(subsystem_ShooterWheels m_flywheels,subsystem_ShooterWrist m_wrist, subsystem_LED m_led){
  return Commands.parallel(new command_ToSetWrist(m_wrist, ()->Constants.ShooterWristConstants.sourceWrist),
  new command_ToFlywheelSpeed(m_flywheels, ()-> Constants.ShooterFlywheelConstants.SourceSpeed), m_led.set_stateCommand("source"));
  }

 public static Command stow(subsystem_ShooterWheels m_flywheels, subsystem_ShooterWrist m_wrist, subsystem_LED m_LED, subsystem_DriveTrain m_drivetrain){
  return Commands.parallel(new command_ToSetWrist(m_wrist,() -> Constants.ShooterWristConstants.minWrist),
  new command_ToFlywheelSpeed(m_flywheels, ()->Constants.ShooterFlywheelConstants.StowSpeed), 
  m_LED.set_stateCommand("stow"));
 }
  // Called when the command is initially scheduled.
  
}
