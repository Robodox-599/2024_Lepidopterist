// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.subsystem_DriveTrain;
import frc.robot.subsystems.subsystem_ShooterWheels;
import frc.robot.subsystems.subsystem_ShooterWrist;

public class command_AutoSpeaker extends Command {
  /** Creates a new command_AutoSpeakerWrist. */
  private final subsystem_ShooterWrist m_Wrist;
  private final subsystem_DriveTrain m_Drive;
  private final subsystem_ShooterWheels m_flywheels;
  private final Timer m_Timer = new Timer();

  public command_AutoSpeaker(subsystem_ShooterWrist shooterWrist, subsystem_DriveTrain drive,
  subsystem_ShooterWheels flywheels) {
    m_Drive = drive;
    m_Wrist = shooterWrist;
    m_flywheels = flywheels;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Wrist);
    addRequirements(m_flywheels);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d shooter2Doffset = new Translation2d(ShooterConstants.blueSpeakerCenter.getX(),
        ShooterConstants.blueSpeakerCenter.getY());
    Translation2d shooterBottomPose = m_Drive.getPose().getTranslation().plus(shooter2Doffset);

    Translation2d speakerCenter2D;
    double deltaZ;

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        speakerCenter2D = new Translation2d(ShooterConstants.redSpeakerCenter.getX(),
            ShooterConstants.redSpeakerCenter.getY());
        deltaZ = ShooterConstants.redSpeakerCenter.getZ() - ShooterConstants.shooterOffset.getZ();
      } else { // assume blue, may cause errors later if something really goes wrong
        speakerCenter2D = new Translation2d(ShooterConstants.blueSpeakerCenter.getX(),
            ShooterConstants.blueSpeakerCenter.getY());
        deltaZ = ShooterConstants.blueSpeakerCenter.getZ() - ShooterConstants.shooterOffset.getZ();
      }
      double fieldDist = speakerCenter2D.getDistance(shooterBottomPose);
      // m_Wrist.set_wrist(Math.atan(deltaZ / fieldDist));
      double newSetpointWrist = Constants.ShooterConstants.a*(Math.pow(fieldDist,5))+
      Constants.ShooterConstants.b*(Math.pow(fieldDist,4))+
      Constants.ShooterConstants.c*(Math.pow(fieldDist,3))+
      Constants.ShooterConstants.d*(Math.pow(fieldDist,2))+
      Constants.ShooterConstants.e*(Math.pow(fieldDist,1))+
      Constants.ShooterConstants.f*(Math.pow(fieldDist,0));
      m_Wrist.set_wrist(newSetpointWrist+Constants.ShooterWristConstants.note_to_wrist_offset);


      double newSetpointFlywheels = ((Constants.ShooterFlywheelConstants.note_max_velocity-Constants.ShooterFlywheelConstants.note_min_velocity)/
      (1+(Math.pow(Math.E, (-Constants.ShooterFlywheelConstants.squish*
      (fieldDist-Constants.ShooterFlywheelConstants.max_range/2)/
      ((Constants.ShooterFlywheelConstants.max_range/2)-Constants.ShooterFlywheelConstants.cushion))))))+Constants.ShooterFlywheelConstants.note_min_velocity;
      //TODO: check for mistakes^


      m_flywheels.set_flywheels(newSetpointFlywheels);
    } else {
      SmartDashboard.putBoolean("Alliance Color Error", false);
    }

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
