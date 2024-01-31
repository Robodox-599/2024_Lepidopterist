// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.subsystem_DriveTrain;
import frc.robot.subsystems.subsystem_Vision;

public class command_DriveTeleop extends Command {
  /** Creates a new command_DriveTeleop. */
  private subsystem_DriveTrain m_DriveTrain;
  private subsystem_Vision m_Vision;
  private DoubleSupplier m_xSpeed;
  private DoubleSupplier m_ySpeed;
  private DoubleSupplier m_zRot;
  private BooleanSupplier m_IsOrientFront;
  private BooleanSupplier m_IsOrientBack;
  private BooleanSupplier m_FieldRelative;
  private BooleanSupplier m_OpenLoop;
    private Pose2d RobotPose;
  private Field2d m_field;

  public command_DriveTeleop(subsystem_DriveTrain driveTrain,
                            subsystem_Vision vision,
                            DoubleSupplier xSpeed,
                            DoubleSupplier ySpeed,
                            DoubleSupplier zRot,
                            BooleanSupplier isOrientFront,
                            BooleanSupplier isOrientBack,
                            BooleanSupplier fieldRelative,
                            BooleanSupplier openLoop) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveTrain = driveTrain;
    m_Vision = vision;
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_zRot = zRot;

    m_IsOrientFront = isOrientFront;
    m_IsOrientBack = isOrientBack;
    m_FieldRelative = fieldRelative;
    m_OpenLoop = openLoop;
    addRequirements(m_DriveTrain);
    addRequirements(m_Vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_field = new Field2d();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DriveTrain.setAutoOrient(m_IsOrientFront.getAsBoolean(), m_IsOrientBack.getAsBoolean(), m_zRot.getAsDouble());
    RobotPose = m_Vision.pullPose2d(m_DriveTrain.getYaw());
    m_field.setRobotPose(RobotPose);
    // m_Vision.smartdashboard(m_field);
    SmartDashboard.putData(m_field);

    // m_DriveTrain.implementVisionPose(m_Vision.getEstimatedGlobalPose());

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
