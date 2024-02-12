// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Subsytem_climb;

public class Command_climbing extends Command {
  /** Creates a new Command_climbing. */
  private final Subsytem_climb m_climb;
  private final DoubleSupplier desired_Position;
  public Command_climbing(Subsytem_climb subsystem, DoubleSupplier given_Position) {

    m_climb = subsystem;
    desired_Position=given_Position;
    addRequirements(m_climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climb.goClimb(desired_Position.getAsDouble());
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
    return false;
  }
}
