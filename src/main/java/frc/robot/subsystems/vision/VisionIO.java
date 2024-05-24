// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
public interface VisionIO {
  @AutoLog
  public static class VisionIOinput {
    public Pose2d estimation = new Pose2d();
    public int tagCount = 0;
    public double timestamps = 0;

  }
  // void readInput(VisionIOinput inputs);
  // void pollNetworkTables();
  public default void updateInputs(VisionIOinput inputs, Pose2d estimate) {}

  public default PhotonPipelineResult getLatestResult() { return null; }

  public default Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) { return null; }

  public default Optional<EstimatedRobotPose> getEstimatedGlobalPose() { return null; }
}
