// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.FlywheelSetpoints;
import frc.robot.Constants.UnitConstants;
import frc.robot.subsystems.subsystem_DriveTrain;
import frc.robot.subsystems.subsystem_Shooter;

public class command_AutoSpeaker extends Command {
  /** Creates a new command_AutoSpeaker. */
  private subsystem_DriveTrain m_Drive;
  private subsystem_Shooter m_Shooter;
  private Translation3d m_speakerCenter;
  
  public command_AutoSpeaker(subsystem_DriveTrain driveTrain, subsystem_Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Shooter = shooter;
    m_Drive = driveTrain;
    addRequirements(m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotConstants.robotColor == null){
      SmartDashboard.putBoolean("Alliance Color Error", true);
      m_speakerCenter = new Translation3d(0, 0, 0);
    }
    m_speakerCenter = (RobotConstants.robotColor == Alliance.Blue) ? 
    FieldConstants.blueSpeakerCenter : FieldConstants.redSpeakerCenter;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = m_Drive.getPose();
    Translation2d shooterBottomPose = robotPose.getTranslation().
    plus(RobotConstants.shooterOffset.toTranslation2d().rotateBy(robotPose.getRotation())); //yeowch

    double fieldDist = m_speakerCenter.toTranslation2d().getDistance(shooterBottomPose);
    double deltaZ = m_speakerCenter.getZ() - RobotConstants.shooterOffset.getZ();
    // double atan2Deg =  Math.atan2(BasePoint[1], BasePoint[0]) * UnitConstants.RAD_TO_DEG;
    // double atan2Normalized = atan2Deg < 0 ? atan2Deg + 180 : atan2Deg - 180;
    SmartDashboard.putNumber("atan2", Math.atan2(deltaZ, fieldDist) * (180.0 / Math.PI));
    m_Shooter.setAbsoluteShootAngle(Math.atan2(deltaZ, fieldDist));
    m_Shooter.setFlywheelSpeed(FlywheelSetpoints.testFlywheelSetpoint);
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
