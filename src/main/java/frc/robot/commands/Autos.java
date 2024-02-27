// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import frc.robot.Constants.AutoConstants;
// import frc.robot.Constants.SwerveConstants;
// import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.subsystem_DriveTrain;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
// import com.choreo.lib.Choreo;
// import com.choreo.lib.ChoreoTrajectory;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  // public static Command exampleAuto(subsystem_DriveTrain driveTrain) {
  //   return Commands.sequence(new command_DriveAuton(driveTrain, () -> {return true;}, "NewPath"),
  //                           new command_DriveAuton(driveTrain, () -> {return false;}, "NewPath1"));
  // }

  // public static Command choreoCommand(subsystem_DriveTrain driveTrain, String trajFilePath){
  //   ChoreoTrajectory trajectory = Choreo.getTrajectory(trajFilePath);
  //   driveTrain.resetOdometry(trajectory.getInitialPose());
  //   AutoConstants.ZPID.enableContinuousInput(-Math.PI, Math.PI);
  //   Command swerveCommand = Choreo.choreoSwerveCommand(trajectory, 
  //                                                     driveTrain::getPose,
  //                                                     AutoConstants.XPID,
  //                                                     AutoConstants.YPID,
  //                                                     AutoConstants.ZPID,
  //                                                     (ChassisSpeeds speeds) -> driveTrain.swerveDrive(
  //                                                       speeds.vxMetersPerSecond,
  //                                                       speeds.vyMetersPerSecond,
  //                                                       speeds.omegaRadiansPerSecond,
  //                                                       false,
  //                                                       false),
  //                                                       () -> {return true;}, 
  //                                                       driveTrain);
  //   return Commands.sequence(
  //     Commands.runOnce(() -> driveTrain.resetOdometry(trajectory.getInitialPose())),
  //     swerveCommand
  //     // driveTrain.run(() -> driveTrain.swerveDrive(0, 0, 0, false, false, true))
  //   );
  // }

  public static Command pathPlannerCommand(subsystem_DriveTrain driveTrain, String autoFilePath){
    PathPlannerPath path = PathPlannerAuto.getPathGroupFromAutoFile(autoFilePath).get(0);
    driveTrain.resetOdometry(path.getPreviewStartingHolonomicPose());
    // driveTrain.setPathPlannerTrajectory(path);
    return new PathPlannerAuto(autoFilePath);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
