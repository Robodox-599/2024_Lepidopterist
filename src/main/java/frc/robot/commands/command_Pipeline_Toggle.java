// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subsystem_Vision;

public class command_Pipeline_Toggle extends Command {
  /** Creates a new command_Pipeline_Toggle. */
  private subsystem_Vision m_Vision;
  public command_Pipeline_Toggle(subsystem_Vision vision) {
    m_Vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_Vision.getPipeline() == 1){
      m_Vision.setPipeline(0);
    }
    else if (m_Vision.getPipeline() == 0){
      m_Vision.setPipeline(1);
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
