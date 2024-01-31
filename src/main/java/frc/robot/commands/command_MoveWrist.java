// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subsystem_Intake;
import frc.robot.subsystems.subsystem_Wrist;
import java.util.function.DoubleSupplier;

public class command_MoveWrist extends Command {
  /** Creates a new command_MoveWrist. */

  subsystem_Wrist m_Wrist;
  DoubleSupplier desiredPos;

  public command_MoveWrist(subsystem_Wrist wristSubsystem, DoubleSupplier desiredPosition) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_Wrist = wristSubsystem;
    desiredPos = desiredPosition;
    addRequirements(m_Wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("entered move wrist", true);
    //m_Wrist.setDesiredWristPos(desiredPos.getAsDouble());
    
    m_Wrist.resetWristPos();
    m_Wrist.setDesiredWristPos(desiredPos.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(m_Wrist.isWristAtDesiredPosition(desiredPos.getAsDouble()) == true){
      return true;
    }
    
    return false;
  }
}
