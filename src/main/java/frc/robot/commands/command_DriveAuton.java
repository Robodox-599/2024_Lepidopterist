// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

// import com.choreo.lib.*;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.SwerveModule;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.subsystem_DriveTrain;

public class command_DriveAuton extends Command {
  /** Creates a new command_DriveAuton. */
  private subsystem_DriveTrain m_DriveTrain;
  private BooleanSupplier m_ToReset;
  private Timer m_Timer;

  private PathPlannerTrajectory m_Trajectory;
  private PathPlannerPath m_Path;
  // private ChoreoTrajectory m_ChoreoTrajectory;
  // private ChoreoTrajectoryState m_ChoreoTrajectoryState;
  // private Choreo m_Choreo;
  private HolonomicDriveController m_DriveController;

  public command_DriveAuton(subsystem_DriveTrain driveTrain,
                            BooleanSupplier toReset,
                            String trajFilePath) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveTrain = driveTrain;
    m_ToReset = toReset;
    
    // m_Path = PathPlannerPath.fromPathFile(trajFilePath);
    m_Path = PathPlannerPath.fromChoreoTrajectory(trajFilePath);
    m_Trajectory = new PathPlannerTrajectory(m_Path, new ChassisSpeeds(0, 0, 0), Rotation2d.fromDegrees(0.0));
    // m_ChoreoTrajectory = Choreo.getTrajectory(trajFilePath);
    // m_ChoreoTrajectoryState = new ChoreoTrajectoryState(0, 0, 0, 0, 0, 0, 0)
    addRequirements(m_DriveTrain);
    m_Timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Timer.stop();
    m_Timer.reset();
    m_Timer.start();

    // Pose2d initialOdo = m_ChoreoTrajectory.getInitialPose();

    Pose2d initOdometry = m_Trajectory.getInitialTargetHolonomicPose();

    if(m_ToReset.getAsBoolean()){
      m_DriveTrain.resetOdometry(initOdometry);
    }

    SmartDashboard.getNumber("Initial_X", initOdometry.getX());
    SmartDashboard.getNumber("Initial_Y", initOdometry.getX());
    SmartDashboard.getNumber("Initial_Heading", initOdometry.getRotation().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Test Case 1
    // ChoreoTrajectoryState statey = m_ChoreoTrajectory.sample(m_Timer.get(), false);
    // ChassisSpeeds chassisSpeeds1 = m_DriveController.calculate(m_DriveTrain.getPose(), 
    //                                                         statey.getPose(), 
    //                                                         Math.hypot(statey.velocityX, statey.velocityY),
    //                                                         Rotation2d.fromRadians(statey.heading));
    
    //Test Case 2
    // ChoreoTrajectoryState stateys = m_ChoreoTrajectory.sample(m_Timer.get(), false);
    // ChassisSpeeds chassisSpeeds2 = stateys.getChassisSpeeds();

    //Default Case
    PathPlannerTrajectory.State state = m_Trajectory.sample(m_Timer.get());
    ChassisSpeeds chassisSpeeds3 = m_DriveController.calculate(m_DriveTrain.getPose(),
                                              state.getTargetHolonomicPose(),
                                              state.velocityMps,
                                              state.targetHolonomicRotation);
    
    SwerveModuleState[] moduleStates = SwerveConstants.kinematics.toSwerveModuleStates(chassisSpeeds3);
    
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, AutoConstants.MaxSpeedMetersPerSecond);
    m_DriveTrain.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrain.swerveDrive(0, 0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Timer.get() >= m_Trajectory.getTotalTimeSeconds();
  }
}
