// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;

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
import frc.robot.Constants.ShooterConstants.WristSepoints;
import frc.robot.Constants.SwerveConstants.DRIVE_STATE;
import frc.robot.Constants.UnitConstants;
import frc.robot.subsystems.subsystem_DriveTrain;
import frc.robot.subsystems.subsystem_Shooter;

public class command_AutoSpeaker extends Command {
  /** Creates a new command_AutoSpeaker. */
  private subsystem_DriveTrain m_Drive;
  private subsystem_Shooter m_Shooter;
  private Translation3d m_speakerCenter;
  private BooleanSupplier m_IsAuto;
  
  public command_AutoSpeaker(subsystem_DriveTrain driveTrain, subsystem_Shooter shooter, BooleanSupplier isAuto) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Shooter = shooter;
    m_Drive = driveTrain;
    m_IsAuto = isAuto;
    addRequirements(m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotConstants.robotColor == null){
      SmartDashboard.putBoolean("Alliance Color Error", true);
      m_speakerCenter = new Translation3d(0, 0, 0);
    } else {
      m_speakerCenter = (RobotConstants.robotColor == Alliance.Blue) ? 
                        FieldConstants.blueSpeakerCenter : 
                        FieldConstants.redSpeakerCenter;
      // m_Drive.setDriveState(DRIVE_STATE.SHOOTER_PREP);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Pose2d robotPose = m_Drive.getPose();
    // Translation2d shooterBottomPose = robotPose.getTranslation().
    // plus(RobotConstants.shooterOffset.toTranslation2d().rotateBy(robotPose.getRotation())); //yeowch
    double fieldDist = m_speakerCenter.toTranslation2d().getDistance(m_Drive.getPose().getTranslation());
    // double deltaZ = m_speakerCenter.getZ() - RobotConstants.shooterOffset.getZ();
    // double atan2Deg =  Math.atan2(deltaZ, fieldDist) * UnitConstants.RAD_TO_DEG;
    // double desiredMotorRot = m_Shooter.shootAngletoMotorRot(atan2Deg);
    // SmartDashboard.putNumber("atan2", atan2Deg);
    // SmartDashboard.putNumber("atan2 motor rot", desiredMotorRot);
    double new_angle = m_Shooter.get_from_table(fieldDist);
    m_Shooter.setDesiredShootAngle(new_angle);
    m_Shooter.setFlywheelSpeed(FlywheelSetpoints.testFlywheelSetpoint);
    if(!m_IsAuto.getAsBoolean()){
      m_Drive.setDriveState(DRIVE_STATE.SHOOTER_PREP);
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
//check which range you are in
    // double d0 = Math.abs(fieldDist- ShooterConstants.WristSepoints.atan_radius_list[0]);
//     double d1 = Math.abs(fieldDist- ShooterConstants.WristSepoints.atan_radius_list[1]);
//     double d2 = Math.abs(fieldDist- ShooterConstants.WristSepoints.atan_radius_list[2]);
//     double d3 = Math.abs(fieldDist- ShooterConstants.WristSepoints.atan_radius_list[3]);
//     double d4 = Math.abs(fieldDist- ShooterConstants.WristSepoints.atan_radius_list[4]);
// if ((d0<d1) && (d0<d2) && (d0<d3) && (d0<d4)){
//     desiredMotorRot = m_Shooter.shootAngletoMotorRot(ShooterConstants.WristSepoints.atan_angle_list[0]);
// }
// if ((d1<d0) && (d1<d2) && (d1<d3) && (d1<d4)){
//     desiredMotorRot = m_Shooter.shootAngletoMotorRot(ShooterConstants.WristSepoints.atan_angle_list[1]);
// }
// if ((d2<d1) && (d2<d0) && (d2<d3) && (d2<d4)){
//     desiredMotorRot = m_Shooter.shootAngletoMotorRot(ShooterConstants.WristSepoints.atan_angle_list[2]);
// }
// if ((d3<d1) && (d3<d2) && (d3<d0) && (d3<d4)){
//     desiredMotorRot = m_Shooter.shootAngletoMotorRot(ShooterConstants.WristSepoints.atan_angle_list[3]);
// }
// if ((d4<d1) && (d4<d2) && (d4<d3) && (d4<d0)){
//     desiredMotorRot = m_Shooter.shootAngletoMotorRot(ShooterConstants.WristSepoints.atan_angle_list[4]);
// }   
    
