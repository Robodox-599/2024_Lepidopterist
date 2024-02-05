// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subsystem_Vision;

public class command_LEDToggle extends Command {
  /** Creates a new command_LEDToggle. */
  private subsystem_Vision m_Vision;
  public command_LEDToggle(subsystem_Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_Vision.getLEDmode() == 3){
      m_Vision.setLEDMode(1);
    }
    else if (m_Vision.getLEDmode() == 1){
      m_Vision.setLEDMode(0);
    }
    else if (m_Vision.getLEDmode() == 0);
    {m_Vision.setLEDMode(3);}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
