// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subsystem_Breakers;
import frc.robot.subsystems.subsystem_Intake;
import frc.robot.subsystems.subsystem_Wrist;

public class command_RunIntake extends Command {
  /** Creates a new command_RunIntake. */
  private final subsystem_Intake m_Intake;
  private final subsystem_Breakers m_Breakers;

  public command_RunIntake(subsystem_Intake intakeSubsystem, subsystem_Breakers breakerSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Intake = intakeSubsystem;
    m_Breakers = breakerSubsystem;
    addRequirements(m_Intake,m_Breakers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Run intake initialized", true);
    m_Intake.runIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Run intake executed", true);
    
  
    //Has not reached indexer
    // m_Intake.runIntake();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(m_Breakers.isBeamBroken()){
      m_Intake.stopIntake();
      return true;
    }

    return false;
  }
}
