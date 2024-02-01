// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.subsystem_DriveTrain;
import frc.robot.Constants;
import frc.robot.subsystems.subsystem_Vision;
import frc.robot.Constants.VisionConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;

public class command_AlignmentTest extends Command {
  /** Creates a new command_AlignmentTest. */
  private subsystem_DriveTrain m_DriveTrain;
  private subsystem_Vision m_Vision;
  private PIDController m_PIDControllerX;
  private PIDController m_PIDControllerY;
  private PIDController m_PIDControllerTheta;
  private PhotonPipelineResult result;
  private double theta_speed;
  private double x_speed;

  public command_AlignmentTest(subsystem_DriveTrain driveTrain, subsystem_Vision vision) {
    m_DriveTrain = driveTrain;
    m_Vision = vision;
    m_PIDControllerX = new PIDController(0.001, 0, 0);
    m_PIDControllerY = new PIDController(0.01, 0, 0);
    m_PIDControllerTheta = new PIDController(0.001, 0, 0);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveTrain, m_Vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_Vision.isDetected()){
      Pose2d robotPose = m_Vision.pullPose2d(m_DriveTrain.getYaw());
      SmartDashboard.putNumber("Vision Robot X", robotPose.getX());
      SmartDashboard.putNumber("Vision Robot Y", robotPose.getY());
      SmartDashboard.putNumber("Vision Robot Rot", robotPose.getRotation().getDegrees());

      Transform3d camToTarget = m_Vision.getVisionResult().getBestTarget().getBestCameraToTarget();
      SmartDashboard.putNumber("Target X", Units.metersToInches(camToTarget.getX()));
      SmartDashboard.putNumber("Target Y", Units.metersToInches(camToTarget.getY()));
      SmartDashboard.putNumber("Yaw", Units.radiansToDegrees(camToTarget.getRotation().getZ()));
      // SmartDashboard.putBoolean("entered execute", true);
      // SmartDashboard.putNumber("Target Z", Units.metersToInches(camToTarget.getZ()));
      // SmartDashboard.putNumber("Roll", Units.radiansToDegrees(camToTarget.getRotation().getX()));
      // SmartDashboard.putNumber("Pitch", Units.radiansToDegrees(camToTarget.getRotation().getY()));
    }
    // double transformedXSpeed = 0.0;
    // double transformedYSpeed = 0.0;
    // double transformedZRot = 0.0;
    // if (m_Vision.m_Detected){
    //   var getThetarad = m_Vision.yaw;
    //   var cameraToTarget = m_Vision.result.getBestTarget().getBestCameraToTarget();
    //   var getCurrentYawRad = m_DriveTrain.getYaw().getDegrees()*Math.PI/180.0;
    //   theta_speed = m_PIDControllerTheta.calculate(getCurrentYawRad,getThetarad);
    //   var getX = m_Vision.result.getBestTarget().getBestCameraToTarget().getX();
    //   x_speed = m_PIDControllerX.calculate(getX,0);

    //    var distanceFromTarget = cameraToTarget.getX();
    //   var xSpeed = m_PIDControllerX.calculate(distanceFromTarget);
    //   if (m_PIDControllerX.atSetpoint()) {
    //     xSpeed = 0;
    //   }

    //   var targetY = cameraToTarget.getY();
    //   var ySpeed = m_PIDControllerY.calculate(targetY);
    //   if (m_PIDControllerY.atSetpoint()) {
    //     ySpeed = 0;
    //   }

    //   var targetYaw = result.getBestTarget().getYaw();
    //   var rSpeed = m_PIDControllerTheta.calculate(targetYaw);
    //   if (m_PIDControllerTheta.atSetpoint()) {
    //     rSpeed = 0;
    //   }
    //   m_DriveTrain.swerveDrive(-xSpeed, -ySpeed, -rSpeed, true, false);

    
    //   // x_speed = m_PIDControllerX.calculate(m_DriveTrain.getPose().getX(),m_Vision.result.getBestTarget().getBestCameraToTarget().getX());
    //   if (getCurrentYawRad>Constants.VisionConstants.threshold){
    //   m_DriveTrain.swerveDrive(0,-x_speed , -theta_speed, true, false);
      
    // } else{
    //   m_DriveTrain.stop();
    // }}
    // else{m_DriveTrain.stop();}
//call swerve command 
//else stop
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
