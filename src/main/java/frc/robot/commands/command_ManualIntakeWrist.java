// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.subsystem_Intake;

public class command_ManualIntakeWrist extends Command {
  /** Creates a new command_ManualIntakeWrist. */
  private subsystem_Intake m_Intake;
  private DoubleSupplier m_Input;

  public command_ManualIntakeWrist(subsystem_Intake intake, DoubleSupplier input) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Intake = intake;
    m_Input = input;
    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double desiredRot = 0.0;

    if(Math.abs(m_Input.getAsDouble()) > ControllerConstants.deadband){
      double temp = desiredRot + m_Input.getAsDouble() * IntakeConstants.maxManualRatio;      
      temp = temp < 0.0 ? 0.0 : temp > 8.3 ? 8.3 : temp;
      desiredRot = temp;
    }

    m_Intake.setDesiredWristPos(desiredRot);
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
