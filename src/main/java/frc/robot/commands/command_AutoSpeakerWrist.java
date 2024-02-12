// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.subsystem_DriveTrain;
import frc.robot.subsystems.subsystem_ShooterWrist;

public class command_AutoSpeakerWrist extends Command {
  /** Creates a new command_AutoSpeakerWrist. */
  private final subsystem_ShooterWrist m_Wrist;
  private final subsystem_DriveTrain m_Drive;
  private Translation3d m_speakerCenter;

  public command_AutoSpeakerWrist(subsystem_ShooterWrist shooterWrist, subsystem_DriveTrain drive) {
    m_Drive = drive;
    m_Wrist = shooterWrist;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotConstants.robotColor==null){
      SmartDashboard.putBoolean("Alliance Color Error", true);
    }
    m_speakerCenter = (RobotConstants.robotColor==Alliance.Blue) ? 
    FieldConstants.blueSpeakerCenter:FieldConstants.redSpeakerCenter;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = m_Drive.getPose();
    Translation2d shooterBottomPose = robotPose.getTranslation().
    plus(RobotConstants.shooterOffset.toTranslation2d().rotateBy(robotPose.getRotation())); //yeowch

    double fieldDist = m_speakerCenter.toTranslation2d().getDistance(shooterBottomPose);
    double deltaZ = m_speakerCenter.getZ() - RobotConstants.shooterOffset.getZ();
    m_Wrist.set_wrist(Math.atan(deltaZ / fieldDist));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
