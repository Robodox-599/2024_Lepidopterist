// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subsystem_DriveTrain;
import frc.robot.subsystems.subsystem_Vision;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.math.util.Units;
// import frc.robot.Constants;
// import frc.robot.RobotContainer;
// import org.photonvision.PhotonCamera;
// import org.photonvision.proto.Photon;
// import org.photonvision.targeting.PhotonPipelineResult;

public class command_AlignmentTest extends Command {
  /** Creates a new command_AlignmentTest. */
  private subsystem_DriveTrain m_DriveTrain;
  private subsystem_Vision m_Vision;
  private PIDController m_PIDControllerX;
  // private PIDController m_PIDControllerY;
  private PIDController m_PIDControllerTheta;
  private double m_ThetaSpeed;
  private double m_XSpeed;

  public command_AlignmentTest(subsystem_DriveTrain driveTrain,
                              subsystem_Vision vision) {
    m_DriveTrain = driveTrain;
    m_Vision = vision;
    m_PIDControllerX = new PIDController(1, 0, 0);
    // m_PIDControllerY = new PIDController(0.01, 0, 0);
    m_PIDControllerTheta = new PIDController(1, 0, 0);
    m_ThetaSpeed = 0.0;
    m_XSpeed = 0.0;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveTrain, m_Vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double transformedXSpeed = 0.0;
    // double transformedYSpeed = 0.0;
    // double transformedZRot = 0.0;
    if (m_Vision.isDetected()){
      double thetaRad = m_Vision.getVisionYaw();
      double currentYawRad = m_DriveTrain.getYaw().getDegrees() * SwerveConstants.DEG_TO_RAD;
      m_ThetaSpeed = -m_PIDControllerTheta.calculate(thetaRad, 0);
      double deltaX = m_Vision.getVisionResult().getBestTarget().getBestCameraToTarget().getX();
      m_XSpeed = m_PIDControllerX.calculate(deltaX, 0);//negative or not
      // x_speed = m_PIDControllerX.calculate(m_DriveTrain.getPose().getX(),m_Vision.result.getBestTarget().getBestCameraToTarget().getX());
      if (Math.abs(currentYawRad) > VisionConstants.threshold){
        m_DriveTrain.swerveDrive(0, m_XSpeed, m_ThetaSpeed, true, false);
      } else{
        m_DriveTrain.swerveDrive(0.0, 0.0, 0.0, true, false);
      }
    } else {
      m_DriveTrain.swerveDrive(0.0, 0.0, 0.0, true, false);
    }
//call swerve command 
//else stop
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
