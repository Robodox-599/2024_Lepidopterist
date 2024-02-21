// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subsystem_ShooterWheels;

public class command_ToFlywheelSpeed extends Command {
  /** Creates a new command_ToFlywheelSpeed. */
  private final subsystem_ShooterWheels m_flywheels;
  private final DoubleSupplier m_target_velocity;
  public command_ToFlywheelSpeed(subsystem_ShooterWheels flywheels, DoubleSupplier target) {

    m_flywheels = flywheels;
    m_target_velocity = target;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_flywheels);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_flywheels.set_flywheels(m_target_velocity.getAsDouble());
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
