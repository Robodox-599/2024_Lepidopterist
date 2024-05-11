// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.subsystem_DriveTrain;

public final class Autos {

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Command runAutoPath(subsystem_DriveTrain driveTrain, String autoFilePath) {
    Pose2d initialPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoFilePath);
    driveTrain.resetOdometry(initialPose);
    driveTrain.overrideGyroAngle(initialPose.getRotation().getDegrees());
    var paths = PathPlannerAuto.getPathGroupFromAutoFile(autoFilePath);
    var finalPath = paths.get(paths.size() - 1).getPathPoses();
    Pose2d finalPose = finalPath.get(finalPath.size() - 1);
    driveTrain.setFinalPose(finalPose);
    if (RobotConstants.robotColor == Alliance.Red) {
      return Commands.sequence(new PathPlannerAuto(autoFilePath), driveTrain.toggleGyroCommand());
    } else {
      return new PathPlannerAuto(autoFilePath);
    }
  }
}
