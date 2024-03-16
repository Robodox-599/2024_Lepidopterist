// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
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
    m_Drive.setDriveState(DRIVE_STATE.SHOOTER_PREP); //TODO: add back once we have some sort of field localization
    Pose2d robotPose = m_Drive.getPose();
    //shooter bottom pose must be applied in a robot relative direction
    Translation2d shooterBottomPose = robotPose.getTranslation().plus(
                                      RobotConstants.shooterOffset.toTranslation2d()
                                      .rotateBy(robotPose.getRotation()));
    
    Translation2d speakerCenter = new Translation2d();

    if(RobotConstants.robotColor != null){
      speakerCenter = RobotConstants.robotColor == Alliance.Blue ? 
                        FieldConstants.blueSpeakerCenter.toTranslation2d() :
                        FieldConstants.redSpeakerCenter.toTranslation2d();
    } else {
      SmartDashboard.putBoolean("Alliance Color Error", true);
    }

    double fieldDist = speakerCenter.getDistance(shooterBottomPose);
    SmartDashboard.putNumber("fielddist", fieldDist);
    double wristSetpoint = quinticCoefficients.a * (Math.pow(fieldDist,5)) +
                            quinticCoefficients.b * (Math.pow(fieldDist,4)) +
                            quinticCoefficients.c * (Math.pow(fieldDist,3)) +
                            quinticCoefficients.d * (Math.pow(fieldDist,2)) +
                            quinticCoefficients.e * (Math.pow(fieldDist,1)) +
                            quinticCoefficients.f * (Math.pow(fieldDist,0));
                            
    SmartDashboard.putNumber("Autospeaker Wrist setpoint", wristSetpoint);
    m_Shooter.setDesiredShootAngle(wristSetpoint);

    double exponent = -sigmoidCoefficients.Kstretch * (fieldDist - sigmoidCoefficients.sigmoidCenter);

    // TODO: Make max - min instead of min - max?
    double flywheelSetpoint = (sigmoidCoefficients.minNoteVelocity - sigmoidCoefficients.maxNoteVelocity) /
                              (1 + Math.pow(Math.E, exponent)) + sigmoidCoefficients.minNoteVelocity;
    SmartDashboard.putNumber("flywheel setpoint", flywheelSetpoint);
    m_Shooter.setFlywheelSpeed(flywheelSetpoint);
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
