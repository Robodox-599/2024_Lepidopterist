// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.subsystem_DriveTrain;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public final class Autos {
  
  public static Command runAutoPath(subsystem_DriveTrain driveTrain, String autoFilePath){
    driveTrain.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile(autoFilePath));
    var paths = PathPlannerAuto.getPathGroupFromAutoFile(autoFilePath);
    var finalPath = paths.get(paths.size() - 1).getPathPoses();
    Pose2d finalPose = finalPath.get(finalPath.size() - 1);
    driveTrain.setFinalPose(finalPose);
    return Commands.sequence(new PathPlannerAuto(autoFilePath),
                            new InstantCommand(() -> driveTrain.overrideGyroAngle(driveTrain.getYaw().getDegrees()), driveTrain));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
