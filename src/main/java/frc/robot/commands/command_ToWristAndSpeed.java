// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.subsystem_Shooter;
import java.util.function.DoubleSupplier;

public class command_ToWristAndSpeed extends Command {
  /** Creates a new command_ToAmpWrist. */
  private final subsystem_Shooter m_Shooter;

  private final DoubleSupplier m_wristSetpoint;
  private final DoubleSupplier m_flywheelSetpoint;
  private final Timer m_wristTimer = new Timer();
  private final Timer m_shooterTimer = new Timer();

  public command_ToWristAndSpeed(
      subsystem_Shooter shooter, DoubleSupplier wristSetpoint, DoubleSupplier flywheelSetpoint) {
    m_Shooter = shooter;
    m_wristSetpoint = wristSetpoint;
    m_flywheelSetpoint = flywheelSetpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wristTimer.start();
    m_shooterTimer.start();
    m_Shooter.setDesiredShootAngle(m_wristSetpoint.getAsDouble());
    m_Shooter.setFlywheelSpeed(m_flywheelSetpoint.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_Shooter.isWristAtDesiredPosition() && m_wristTimer.get() <= 0.5) {
      m_wristTimer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_wristTimer.get() >= ShooterConstants.wristTestTime
        && m_Shooter.flywheelsAtDesiredSpeed();
  }
}
