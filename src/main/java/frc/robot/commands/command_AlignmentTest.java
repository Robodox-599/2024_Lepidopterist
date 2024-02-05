// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.subsystem_DriveTrain;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.subsystem_Vision;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;


public class command_AlignmentTest extends Command {
  /** Creates a new command_AlignmentTest. */
  private subsystem_DriveTrain m_DriveTrain;
  private subsystem_Vision m_Vision;
  // private PIDController m_PIDControllerX;
  // private PIDController m_PIDControllerY;
  // private PIDController m_PIDControllerTheta;
  // private PhotonPipelineResult result;
  // private double theta_speed;
  // private double x_speed;

  public command_AlignmentTest(subsystem_DriveTrain driveTrain, subsystem_Vision vision) {
    m_DriveTrain = driveTrain;
    m_Vision = vision;
    // m_PIDControllerX = new PIDController(0.001, 0, 0);
    // m_PIDControllerY = new PIDController(0.01, 0, 0);
    // m_PIDControllerTheta = new PIDController(0.001, 0, 0);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveTrain, m_Vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_Vision.LLhasTargets()){
      double[] bot_pose = m_Vision.getTable();
      SmartDashboard.putNumber("Field X", bot_pose[0]);
      SmartDashboard.putNumber("Field Y", bot_pose[1]);
      SmartDashboard.putNumber("Field Z", bot_pose[5]+360 %360);
      SmartDashboard.putNumber("1",1);}
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
