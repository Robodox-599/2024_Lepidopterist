// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.UnitConstants;
import frc.robot.Constants.SwerveConstants.DRIVE_STATE;
import frc.robot.subsystems.subsystem_DriveTrain;

public class command_DriveTeleop extends Command {
  /** Creates a new command_DriveTeleop. */
  private subsystem_DriveTrain m_DriveTrain;
  private DoubleSupplier m_xSpeed;
  private DoubleSupplier m_ySpeed;
  private DoubleSupplier m_zRot;
  private BooleanSupplier m_IsOrientFront;
  private BooleanSupplier m_IsOrientBack;
  private BooleanSupplier m_FieldRelative;
  private BooleanSupplier m_OpenLoop;

  public command_DriveTeleop(subsystem_DriveTrain driveTrain,
                            DoubleSupplier xSpeed,
                            DoubleSupplier ySpeed,
                            DoubleSupplier zRot,
                            BooleanSupplier isOrientFront,
                            BooleanSupplier isOrientBack,
                            BooleanSupplier fieldRelative,
                            BooleanSupplier openLoop) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveTrain = driveTrain;
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_zRot = zRot;
    m_IsOrientFront = isOrientFront;
    m_IsOrientBack = isOrientBack;
    m_FieldRelative = fieldRelative;
    m_OpenLoop = openLoop;
    addRequirements(m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Field2d speakerAsField = new Field2d();
    Pose2d speakerAsPose = new Pose2d(FieldConstants.blueSpeakerCenter.getX(), FieldConstants.blueSpeakerCenter.getY(), new Rotation2d(0));
    speakerAsField.setRobotPose(speakerAsPose.div(UnitConstants.METRES_TO_FEETRES));
    SmartDashboard.putData("Speaker Position", speakerAsField);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_DriveTrain.setAutoOrient(m_IsOrientFront.getAsBoolean(), m_IsOrientBack.getAsBoolean(), m_zRot.getAsDouble());
    
    double transformedXSpeed = 0.0;
    double transformedYSpeed = 0.0;
    double transformedZRot = 0.0;

    if(Math.abs(m_xSpeed.getAsDouble()) > ControllerConstants.deadband){
      transformedXSpeed = m_DriveTrain.setThrottle(2.0 * m_xSpeed.getAsDouble());
    }

    if(Math.abs(m_ySpeed.getAsDouble()) > ControllerConstants.deadband){
      transformedYSpeed = m_DriveTrain.setThrottle(2.0 * m_ySpeed.getAsDouble());
    }

    if(Math.abs(m_zRot.getAsDouble()) > ControllerConstants.deadband){
      transformedZRot = m_DriveTrain.setThrottle(2.0 * m_zRot.getAsDouble());
    }
    
    if (m_DriveTrain.getDriveState() == DRIVE_STATE.SHOOTER_PREP){
      Pose2d drivePose = m_DriveTrain.getPose();
      double deltaX = drivePose.getX();
      double deltaY = drivePose.getY();

      Optional<Alliance> ally = DriverStation.getAlliance();
      if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
          deltaX-=FieldConstants.redSpeakerCenter.getX();
          deltaY-=FieldConstants.redSpeakerCenter.getY();
          SmartDashboard.putNumber("deltaY", deltaY);
          SmartDashboard.putNumber("deltaX", deltaX);
          SmartDashboard.putNumber("raw atan (deg)", Math.atan2(deltaY, deltaX) * UnitConstants.RAD_TO_DEG);
          m_DriveTrain.setDesiredAngle(Math.atan2(deltaY, deltaX) * UnitConstants.RAD_TO_DEG);
        } else if (ally.get() == Alliance.Blue) {
          deltaX-=FieldConstants.blueSpeakerCenter.getX();
          deltaY-=FieldConstants.blueSpeakerCenter.getY();
          SmartDashboard.putNumber("deltaY", deltaY);
          SmartDashboard.putNumber("deltaX", deltaX);
          SmartDashboard.putNumber("raw atan (deg)", Math.atan2(deltaY, deltaX) * UnitConstants.RAD_TO_DEG);
          double tempDangle = 180 + Math.atan2(deltaY, deltaX) * UnitConstants.RAD_TO_DEG;
          m_DriveTrain.setDesiredAngle(tempDangle >= 180.0 ? tempDangle - 360.0 : tempDangle);
        } else {
          SmartDashboard.putBoolean("Alliance Color Error", true);
        }
    } else {
      SmartDashboard.putBoolean("Alliance Color Error", true);
    }
  }

    SmartDashboard.putNumber("X Speed", transformedXSpeed);
    SmartDashboard.putNumber("Y Speed", transformedYSpeed);
    SmartDashboard.putNumber("Z Rot", transformedZRot);

    m_DriveTrain.swerveDrive(transformedXSpeed * SwerveConstants.maxSpeed,
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
