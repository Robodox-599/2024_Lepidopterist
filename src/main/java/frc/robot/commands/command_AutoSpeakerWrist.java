// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.subsystem_ShooterWrist;

public class command_AutoSpeakerWrist extends Command {
  /** Creates a new command_AutoSpeakerWrist. */
  private final subsystem_ShooterWrist m_Wrist;
  //private final subsystem_Drive m_Drive;
  private final Timer m_Timer = new Timer();

  public command_AutoSpeakerWrist(subsystem_ShooterWrist shooterWrist/*, subsystem_Drive drive*/) {
    // m_Drive = drive;
    m_Wrist = shooterWrist;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d m_DrivePose = m_Drive.getPose2d();
    double fieldDist = ShooterConstants.speakerCenter.getDistance(m_DrivePose.getTranslation());
    m_Wrist.set_wrist(Math.atan((speakerCenterZ-robotz)/fieldDist))
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
