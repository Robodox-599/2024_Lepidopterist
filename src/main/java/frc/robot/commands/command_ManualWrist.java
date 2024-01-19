// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.subsystem_ShooterWrist;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterWristConstants;

import edu.wpi.first.wpilibj2.command.Command;

public class command_ManualWrist extends Command {
  /** Creates a new command_ManualWrist. */
  private final subsystem_ShooterWrist m_wrist;
  private final DoubleSupplier m_stickPos;
  public command_ManualWrist(subsystem_ShooterWrist wrist, DoubleSupplier stick) {
    m_wrist = wrist;
    m_stickPos = stick;
    addRequirements(m_wrist);
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     if (Math.abs(m_stickPos.getAsDouble()) > OperatorConstants.manual_wrist_threshold){
      m_wrist.set_wrist(m_stickPos.getAsDouble());
    }
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
