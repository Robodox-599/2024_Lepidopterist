// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.WristSepoints;
import frc.robot.subsystems.shooter.subsystem_Shooter;
import java.util.function.DoubleSupplier;

public class command_ManualShooter extends Command {
  /** Creates a new command_ManualShooter. */
  subsystem_Shooter m_Shooter;

  DoubleSupplier m_Input;

  public command_ManualShooter(subsystem_Shooter shooter, DoubleSupplier input) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Shooter = shooter;
    m_Input = input;
    addRequirements(m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double m_ShootRot = m_Shooter.getWristPos();
    SmartDashboard.putNumber("Shooter Manual Input", m_Input.getAsDouble());
    if (Math.abs(m_Input.getAsDouble()) > ControllerConstants.deadband) {
      m_ShootRot += m_Input.getAsDouble() * ShooterConstants.max_manual_ratio;
      double m_ShootAngle = m_Shooter.MotorRotToShootAngle(m_ShootRot);
      if (m_ShootAngle <= WristSepoints.maxShootAngle) {
        m_Shooter.setDesiredShootAngle(m_ShootAngle);
      }
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
