// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.DRIVE_STATE;
import frc.robot.Constants.UnitConstants;
import frc.robot.subsystems.subsystem_DriveTrain;
import frc.robot.subsystems.subsystem_Vision;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class command_DriveTeleop extends Command {
  /** Creates a new command_DriveTeleop. */
  private subsystem_DriveTrain m_DriveTrain;

  private subsystem_Vision m_Vision;
  private DoubleSupplier m_xSpeed;
  private DoubleSupplier m_ySpeed;
  private DoubleSupplier m_zRot;
  private BooleanSupplier m_FieldRelative;
  private BooleanSupplier m_OpenLoop;
  private BooleanSupplier m_invert;
  private Translation2d m_speakerCenter = new Translation2d();

  public command_DriveTeleop(
      subsystem_DriveTrain driveTrain,
      subsystem_Vision vision,
      DoubleSupplier xSpeed,
      DoubleSupplier ySpeed,
      DoubleSupplier zRot,
      BooleanSupplier fieldRelative,
      BooleanSupplier openLoop) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveTrain = driveTrain;
    m_Vision = vision;
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_zRot = zRot;
    m_FieldRelative = fieldRelative;
    m_OpenLoop = openLoop;
    addRequirements(m_DriveTrain, m_Vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotConstants.robotColor == null) {
      SmartDashboard.putBoolean("Alliance Color Error", true);
    }
    m_speakerCenter =
        (RobotConstants.robotColor == Alliance.Blue)
            ? FieldConstants.blueSpeakerCenter.toTranslation2d()
            : FieldConstants.redSpeakerCenter.toTranslation2d();

    Field2d speakerAsField = new Field2d();
    Pose2d speakerAsPose = new Pose2d(m_speakerCenter, Rotation2d.fromDegrees(0));
    speakerAsField.setRobotPose(speakerAsPose.div(UnitConstants.METRES_TO_FEETRES));
    SmartDashboard.putData("Speaker Position", speakerAsField);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: Add back in once vision can see
    // if(m_Vision.hasTargets()){
    //   Pose2d robotPose = m_Vision.getEstimatedPose();
    //   SmartDashboard.putNumber("Vision Robot X", robotPose.getX());
    //   SmartDashboard.putNumber("Vision Robot Y", robotPose.getY());
    //   SmartDashboard.putNumber("Vision Robot Rot", robotPose.getRotation().getDegrees());
    //   m_DriveTrain.implementVisionPose(robotPose, m_Vision.getTimestampMS());
    // }

    double transformedXSpeed = 0.0;
    double transformedYSpeed = 0.0;
    double transformedZRot = 0.0;
    double sign = m_DriveTrain.getGyroToggle() ? -1 : 1;

    if (Math.abs(m_xSpeed.getAsDouble()) > ControllerConstants.deadband) {
      transformedXSpeed = m_DriveTrain.setLinearThrottle(sign * 2.0 * m_xSpeed.getAsDouble());
    }

    if (Math.abs(m_ySpeed.getAsDouble()) > ControllerConstants.deadband) {
      transformedYSpeed = m_DriveTrain.setLinearThrottle(sign * 2.0 * m_ySpeed.getAsDouble());
    }

    if (Math.abs(m_zRot.getAsDouble()) > ControllerConstants.deadband) {
      transformedZRot = m_DriveTrain.setAngularThrottle(2.0 * m_zRot.getAsDouble());
    }

    if (m_DriveTrain.getDriveState() == DRIVE_STATE.SHOOTER_PREP) {
      Pose2d drivePose = m_DriveTrain.getPose();
      double deltaX = drivePose.getX() - m_speakerCenter.getX();
      double deltaY = drivePose.getY() - m_speakerCenter.getY();
      SmartDashboard.putNumber("deltaY", deltaY);
      SmartDashboard.putNumber("deltaX", deltaX);
      SmartDashboard.putNumber(
          "raw atan (deg)", Math.atan2(deltaY, deltaX) * UnitConstants.RAD_TO_DEG);
      if (RobotConstants.robotColor == Alliance.Red) {
        m_DriveTrain.setDesiredAngle(Math.atan2(deltaY, deltaX) * UnitConstants.RAD_TO_DEG);
      } else if (RobotConstants.robotColor == Alliance.Blue) {
        double tempDangle = 180 + Math.atan2(deltaY, deltaX) * UnitConstants.RAD_TO_DEG;
        m_DriveTrain.setDesiredAngle(tempDangle >= 180.0 ? tempDangle - 360.0 : tempDangle);
      } else {
        SmartDashboard.putBoolean("Alliance Color Error", true);
      }
    }

    m_DriveTrain.swerveDrive(
        transformedXSpeed * SwerveConstants.maxSpeed,
        transformedYSpeed * SwerveConstants.maxSpeed,
        transformedZRot * SwerveConstants.maxAngularVelocity,
        m_FieldRelative.getAsBoolean(),
        m_OpenLoop.getAsBoolean());
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
