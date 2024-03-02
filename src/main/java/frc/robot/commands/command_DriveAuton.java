// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
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
  private ChoreoTrajectory m_ChoreoTrajectory;
  // private ChoreoTrajectoryState m_ChoreoTrajectoryState;
  // private Choreo m_Choreo;
  private HolonomicDriveController m_DriveController;

  public command_DriveAuton(subsystem_DriveTrain driveTrain,
                            BooleanSupplier toReset,
                            String trajFilePath) {
    // Use addRequirements() here to declare subsystem dependencies.
    // SmartDashboard.putBoolean("Constructor", true);
    m_DriveTrain = driveTrain;
    m_ToReset = toReset;
    // SmartDashboard.putBoolean("Line 49", true);
    // m_Path = PathPlannerPath.fromPathFile(trajFilePath);
    // m_Path = PathPlannerPath.fromChoreoTrajectory(trajFilePath);
    m_ChoreoTrajectory = Choreo.getTrajectory(trajFilePath);
    // SmartDashboard.putBoolean("Line 52", true);
    m_DriveController = new HolonomicDriveController(AutoConstants.XPID, 
                                                    AutoConstants.YPID, 
                                                    AutoConstants.ThetaPIDRadians);
    
    // ChoreoTrajectoryState initialState = m_ChoreoTrajectory.getInitialState();
    // SmartDashboard.putNumber("Constructor1", 4);
    // SMar
    // SmartDashboard.putBoolean("isNull", initialState.getChassisSpeeds().equals(new ChassisSpeeds()));
    // SmartDashboard.putBoolean("isNull2", initialState.getPose().equals(new Pose2d()));
    // m_Trajectory = new PathPlannerTrajectory(m_Path, 
    //                                         initialState.getChassisSpeeds(),
    //                                         initialState.getPose().getRotation());
    // SmartDashboard.putNumber("Constructor2", 3);
    // m_Trajectory = new PathPlannerTrajectory(m_Path, new ChassisSpeeds(), new Rotation2d());
    // m_Trajectory = new PathPlannerTrajectory(m_Path, new ChassisSpeeds(0, 0, 0), Rotation2d.fromDegrees(0.0));
    // ChoreoTrajectoryState initialState = m_ChoreoTrajectory.getInitialState();
    // ChassisSpeeds initialSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(null, null)
    // m_Trajectory = new PathPlannerTrajectory(m_Path, initialSpeeds, null)
    // SmartDashboard.putBoolean("Line 54", true);
    // m_ChoreoTrajectoryState = new ChoreoTrajectoryState(0, 0, 0, 0, 0, 0, 0)
    // m_Trajectory = PathPlannerPath.fromChoreoTrajectory(trajFilePath);
    addRequirements(m_DriveTrain);
    m_Timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartDashboard.putBoolean("Initialize", true);
    m_DriveTrain.setTrajectory(m_ChoreoTrajectory);
    m_Timer.stop();
    m_Timer.reset();
    m_Timer.start();
    SmartDashboard.putNumber("Total Time", m_ChoreoTrajectory.getTotalTime());

    // Pose2d initialOdo = m_ChoreoTrajectory.getInitialPose();

    // Pose2d initialOdometry = m_Trajectory.getInitialTargetHolonomicPose();
    Pose2d initialOdometry = m_ChoreoTrajectory.getInitialPose();
    // m_Trajectory.getInitialTargetHolonomicPose()

    if(m_ToReset.getAsBoolean()){
      // m_DriveTrain.resetOdometry(initOdometry);
      m_DriveTrain.resetOdometry(initialOdometry);
    }

    // SmartDashboard.getNumber("Initial_X", initOdometry.getX());
    // SmartDashboard.getNumber("Initial_Y", initOdometry.getX());
    // SmartDashboard.getNumber("Initial_Heading", initOdometry.getRotation().getDegrees());
    SmartDashboard.getNumber("Initial_X", initialOdometry.getX());
    SmartDashboard.getNumber("Initial_Y", initialOdometry.getX());
    SmartDashboard.getNumber("Initial_Heading", initialOdometry.getRotation().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double[] flags = new double[7];
    // SmartDashboard.putBoolean("Execute", true);
    ChoreoTrajectoryState state = m_ChoreoTrajectory.sample(m_Timer.get());
    // PathPlannerTrajectory.State state = m_Trajectory.sample(m_Timer.get());

    ChassisSpeeds chassisSpeeds = m_DriveController.calculate(m_DriveTrain.getPose(),
                                                              state.getPose(),
                                                              Math.hypot(state.velocityX, state.velocityY),
                                                              Rotation2d.fromRadians(state.heading));

    // SmartDashboard.putNumber("Auton Angular Velocity (Deg/s)", statey.angularVelocity * SwerveConstants.RAD_TO_DEG);
    // SmartDashboard.putNumber("Auton Heading (Deg)", statey.heading * SwerveConstants.RAD_TO_DEG);

    // ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(statey.velocityX, 
    //                                                                     statey.velocityY, 
    //                                                                     statey.angularVelocity,
    //                                                                     Rotation2d.fromRadians(statey.heading));

    SwerveModuleState[] moduleStates = SwerveConstants.kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, AutoConstants.MaxSpeedMetersPerSecond);
    m_DriveTrain.setModuleStates(moduleStates);
    /* 
    // Test Case 1
    // ChoreoTrajectoryState statey = m_ChoreoTrajectory.sample(m_Timer.get(), false);
    // ChassisSpeeds chassisSpeeds1 = m_DriveController.calculate(m_DriveTrain.getPose(), 
    //                                                         statey.getPose(), 
    //                                                         Math.hypot(statey.velocityX, statey.velocityY),
    //                                                         Rotation2d.fromRadians(statey.heading));
    
    // Test Case 2
    // ChoreoTrajectoryState stateys = m_ChoreoTrajectory.sample(m_Timer.get(), false);
    // ChassisSpeeds chassisSpeeds2 = stateys.getChassisSpeeds();

    //Default Case
    // PathPlannerTrajectory.State state = m_Trajectory.sample(m_Timer.get());
    // SmartDashboard.putNumber("Line 112", flags[0]);
    // flags[0]++;
    
    // double velocity = Math.signum(statey.velocityY) * Math.hypot(statey.velocityX, statey.velocityY);
    // ChassisSpeeds chassisSpeeds3 = m_DriveController.calculate(m_DriveTrain.getPose(),
    //                                           state.getTargetHolonomicPose(),
    //                                           state.velocityMps,
    //                                           state.targetHolonomicRotation);
    // SmartDashboard.putNumber("Line 118", flags[1]);
    // flags[1]++;
    // ChassisSpeeds chassisSpeeds = m_DriveController.calculate(m_DriveTrain.getPose(),
    //                                                           statey.getPose(),
    //                                                           velocity, 
    //                                                           Rotation2d.fromRadians(statey.heading));
    // SmartDashboard.putNumber("Line 123", flags[0]);
    // flags[0]++;
    // SmartDashboard.putNumber("Line 125", flags[1]);
    // flags[1]++;
    // SmartDashboard.putNumber("Line 127", flags[2]);
    // flags[2]++;
    // SmartDashboard.putNumber("Line 129", flags[3]);
    // flags[3]++;
    */
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrain.swerveDrive(0, 0, 0, false, false, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return m_Timer.get() >= m_ChoreoTrajectory.getFinalState().timestamp;
    // return m_ChoreoTrajectory.sample(m_Timer.get()).equals(m_ChoreoTrajectory.getFinalState());
    Pose2d currentPose = m_DriveTrain.getPose();
    Pose2d finalPose = m_ChoreoTrajectory.getFinalPose();
    
    boolean x_threshold = Math.abs((currentPose.getX() - finalPose.getX()) / finalPose.getX()) <= 0.05;
    boolean y_threshold = Math.abs((currentPose.getY() - finalPose.getY()) / finalPose.getY()) <= 0.05;
    boolean theta_threshold = Math.abs((currentPose.getRotation().getDegrees() - finalPose.getRotation().getDegrees()) / finalPose.getRotation().getDegrees()) <= 0.05;
    
    return (m_Timer.get() >= m_ChoreoTrajectory.getTotalTime() &&
          y_threshold && theta_threshold) || x_threshold;
    // return m_Timer.get() >= m_ChoreoTrajectory.getTotalTime();
  }
}
