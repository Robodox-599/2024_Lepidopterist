// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterWristConstants;
import frc.robot.subsystems.subsystem_ShooterWrist;

public class command_ToAmpWrist extends Command {
  /** Creates a new command_ToAmpWrist. */
  private final subsystem_ShooterWrist m_Wrist;
  private final double m_setpoint;
  private final Timer m_Timer = new Timer();

  public command_ToAmpWrist(subsystem_ShooterWrist shooterWrist, DoubleSupplier setpoint) {
    m_Wrist = shooterWrist;
    m_setpoint = setpoint.getAsDouble();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Timer.start();
    m_Wrist.set_wrist(m_setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_Wrist.isWristAtDesiredPosition())
      m_Timer.reset();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_Timer.get() < ShooterWristConstants.targetTime);
  }
}
