// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
// import java.util.Optional;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.InstantCommand;

public final class Autos {

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Command runAutoPath(Drive drive, String autoFilePath) {
    Pose2d initialPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoFilePath);
    drive.resetOdometry(initialPose);
    drive.overrideGyroAngle(initialPose.getRotation().getDegrees());
    var paths = PathPlannerAuto.getPathGroupFromAutoFile(autoFilePath);
    var finalPath = paths.get(paths.size() - 1).getPathPoses();
    Pose2d finalPose = finalPath.get(finalPath.size() - 1);
    drive.setFinalPose(finalPose);
    return new PathPlannerAuto(autoFilePath);

    // if (RobotConstants.robotColor == Alliance.Red) {
    //   return Commands.sequence(new PathPlannerAuto(autoFilePath), drive.toggleGyroCommand());
    // } else {
    //   return new PathPlannerAuto(autoFilePath);
    // }
  }
}
