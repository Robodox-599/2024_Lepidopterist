// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.subsystem_Breakers;
import frc.robot.subsystems.subsystem_Indexer;

public class command_RunIndexer extends Command {

  private final subsystem_Indexer m_Indexer;
  private final subsystem_Breakers m_Breakers;

  /** Creates a new command_RunIndexer. */
  public command_RunIndexer(subsystem_Indexer indexerSubsystem, subsystem_Breakers breakerSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_Indexer = indexerSubsystem;
    m_Breakers = breakerSubsystem;

    addRequirements(m_Indexer,m_Breakers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Indexer.runIndexer();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(m_Breakers.isSecondBeamBroken()){
      m_Indexer.stopIndexer();
      return true;
    }

    return false;
  }
}
