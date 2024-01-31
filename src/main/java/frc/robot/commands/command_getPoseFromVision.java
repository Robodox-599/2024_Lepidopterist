// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subsystem_DriveTrain;
import frc.robot.subsystems.subsystem_Vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class command_getPoseFromVision extends Command {
  /** Creates a new command_getPoseFromVision. */
  private subsystem_DriveTrain m_drive;
  private subsystem_Vision m_Vision;
  private Pose2d RobotPose;
  private Field2d m_field;

  public command_getPoseFromVision(subsystem_DriveTrain drive, subsystem_Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
  m_drive = drive;
  m_Vision = vision;
  addRequirements(m_Vision);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotPose = m_Vision.pullPose2d(m_drive.getYaw());
    m_field.setRobotPose(RobotPose);
    // m_Vision.smartdashboard(m_field);
    SmartDashboard.putData(m_field);


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
