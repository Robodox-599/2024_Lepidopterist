// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.sigmoidCoefficients;
import frc.robot.Constants.SwerveConstants.DRIVE_STATE;
import frc.robot.Constants.ShooterConstants.quinticCoefficients;
import frc.robot.subsystems.subsystem_DriveTrain;
import frc.robot.subsystems.subsystem_Shooter;

/** Constantly aligns wrist and drivebase to speaker, never ending */
public class command_AutoSpeaker extends Command {
  /** Creates a new command_AutoSpeakerWrist. */
  private subsystem_DriveTrain m_Drive;
  private subsystem_Shooter m_Shooter;

  public command_AutoSpeaker(subsystem_DriveTrain drive, subsystem_Shooter shooter) {
    m_Drive = drive;
    m_Shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Drive.setDriveState(DRIVE_STATE.SHOOTER_PREP);
    
    Pose2d robotPose = m_Drive.getPose();
    //shooter bottom pose must be applied in a robot relative direction
    Translation2d shooterBottomPose = m_Drive.getPose().getTranslation().plus(
                                  RobotConstants.shooterOffset.toTranslation2d().rotateBy(robotPose.getRotation()));
    
    Translation2d speakerCenter2D = new Translation2d(ShooterConstants.blueSpeakerCenter.getX(),
                                                      ShooterConstants.blueSpeakerCenter.getY());

    if (RobotConstants.robotColor == Alliance.Red) {
      speakerCenter2D = new Translation2d(ShooterConstants.redSpeakerCenter.getX(),
                                          ShooterConstants.redSpeakerCenter.getY());
    } else if (RobotConstants.robotColor != Alliance.Blue){
      SmartDashboard.putBoolean("Alliance Color Error", true);
    }

      double fieldDist = speakerCenter2D.getDistance(shooterBottomPose);

      double wristSetpoint = quinticCoefficients.a * (Math.pow(fieldDist,5)) +
                                quinticCoefficients.b * (Math.pow(fieldDist,4)) +
                                quinticCoefficients.c * (Math.pow(fieldDist,3)) +
                                quinticCoefficients.d * (Math.pow(fieldDist,2)) +
                                quinticCoefficients.e * (Math.pow(fieldDist,1)) +
                                quinticCoefficients.f * (Math.pow(fieldDist,0));

      m_Shooter.setDesiredShootAngle(ShooterConstants.testSpeakerWrist);

      double exponent = -sigmoidCoefficients.Kstretch * (fieldDist - sigmoidCoefficients.sigmoidCenter);

      double flywheelSetpoint = 
      (sigmoidCoefficients.minNoteVelocity - sigmoidCoefficients.maxNoteVelocity) /
      (1 + Math.pow(Math.E, exponent)) + sigmoidCoefficients.minNoteVelocity;

      m_Shooter.setFlywheelSpeed(ShooterConstants.testFlywheelSetpoint);

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
