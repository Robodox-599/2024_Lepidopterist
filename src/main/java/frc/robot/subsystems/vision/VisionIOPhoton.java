package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionConstants.VisionBasic;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
// import org.photonvision.simulation.PhotonCameraSim;
// import org.photonvision.simulation.SimCameraProperties;
// import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhoton implements VisionIO {
  private final PhotonCamera camera;
  private final String name;
  private final PhotonPoseEstimator odometry;
  private double pastTimestamp;
  private Transform3d pose;
  public List<PhotonTrackedTarget> targets;
  // private static DriverStation.Alliance storedAlliance = DriverStation.Alliance.Invalid;

  // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

  public VisionIOPhoton(String name, Transform3d pose) {
    this.name = name;
    this.pose = pose;
    camera = new PhotonCamera(VisionBasic.kCameraName);

    odometry =
        new PhotonPoseEstimator(
            VisionBasic.kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_RIO,
            camera,
            VisionBasic.kRobotToCam);
    odometry.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // ----- Simulation
    if (RobotBase.isSimulation()) {
    //Create the vision system simulation which handles cameras and targets on the field.
      visionSim = new VisionSystemSim("main");
    //Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      visionSim.addAprilTags(VisionBasic.kTagLayout);
    //Create simulated camera properties. These can be set to mimic your actual camera.
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
      cameraProp.setCalibError(0.35, 0.10);
      cameraProp.setFPS(15);
      cameraProp.setAvgLatencyMs(50);
      cameraProp.setLatencyStdDevMs(15);
    // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
    // targets.
      cameraSim = new PhotonCameraSim(camera, cameraProp);
    // Add the simulated camera to view the targets on this simulated field.
          visionSim.addCamera(cameraSim, VisionBasic.kRobotToCam);

          cameraSim.enableDrawWireframe(true);
        }
  }

  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      var visionEst = odometry.update();
      double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
      boolean newResult = Math.abs(latestTimestamp - pastTimestamp) > 1e-5;
      if (RobotBase.isSimulation()) {
        visionEst.ifPresentOrElse(
            est ->
                getSimDebugField()
                    .getObject("VisionEstimation")
                    .setPose(est.estimatedPose.toPose2d()),
            () -> {
              if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
            });
      }
      if (newResult) pastTimestamp = latestTimestamp;
      return visionEst;
    }

     /**
      * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for
         use
      * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
        SwerveDrivePoseEstimator}.
      * This should only be used when there are targets visible.
      *
      * @param estimatedPose The estimated pose to guess standard deviations for.
      */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
      var estStdDevs = VisionBasic.kSingleTagStdDevs;
      var targets = getLatestResult().getTargets();
      int numTags = 0;
      double avgDist = 0;
      for (var tgt : targets) {
        var tagPose = odometry.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist +=
  
  tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
      }
      if (numTags == 0) return estStdDevs;
      avgDist /= numTags;
      // Decrease std devs if multiple targets are visible
      if (numTags > 1) estStdDevs = VisionBasic.kMultiTagStdDevs;
      // Increase std devs based on (average) distance
      if (numTags == 1 && avgDist > 4)
        estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

      return estStdDevs;
    }

  // ----- Simulation

  @Override
  public void updateInputs(VisionIOinput inputs) {
    PhotonPipelineResult result = camera.getLatestResult();
    Optional<EstimatedRobotPose> currentPose =
        checkValidResult(result.targets) ? odometry.update(result) : Optional.empty();

    // set if PhotonVision has a target
    if (result.hasTargets() && currentPose.isPresent()) {
      inputs.hasTarget = true;
      targets = currentPose.get().targetsUsed;
    } else {
      inputs.hasTarget = false;
    }

    inputs.isNew = false;

    if (currentPose.isPresent() && targets != null) {
      if (targets.size() > 1) {
        double minDistance = Double.MAX_VALUE;
        double maxDistance = 0.0;
        double maxAmbiguity = 0.0;
        for (PhotonTrackedTarget target : targets) {
          double distance = target.getBestCameraToTarget().getTranslation().getNorm();
          double ambiguity = target.getPoseAmbiguity();
          if (distance < minDistance) {
            minDistance = distance;
          }
          if (distance > maxDistance) {
            maxDistance = distance;
          }
          if (ambiguity > maxAmbiguity) {
            maxAmbiguity = ambiguity;
          }
        }
        inputs.minDistance = minDistance;
        inputs.maxDistance = maxDistance;
        inputs.maxAmbiguity = maxAmbiguity;
      } else {
        inputs.minDistance = targets.get(0).getBestCameraToTarget().getTranslation().getNorm();
        inputs.maxDistance = inputs.minDistance;
        inputs.maxAmbiguity = targets.get(0).getPoseAmbiguity();
      }

      inputs.singleIDUsed = targets.get(0).getFiducialId();
      inputs.singleIDUsedDouble = inputs.singleIDUsed;

      if (inputs.maxAmbiguity < 0.7) {
        // sets inputs
        inputs.timestamp = currentPose.get().timestampSeconds;

        if (pastTimestamp != inputs.timestamp) {
          inputs.isNew = true;
        }

        inputs.x = currentPose.get().estimatedPose.getX();
        inputs.y = currentPose.get().estimatedPose.getY();
        inputs.rotation = currentPose.get().estimatedPose.getRotation().getAngle();
      }
    }

    pastTimestamp = inputs.timestamp;

    odometry.setFieldTags(VisionBasic.kTagLayout);
  }

  @Override
  public String getName() {
    return name;
  }

  public void setReferencePose(Pose2d pose) {
    odometry.setReferencePose(pose);
  }

   public void simulationPeriodic(Pose2d robotSimPose) {
       visionSim.update(robotSimPose);
       Logger.recordOutput("Vision/SimIO/updateSimPose", robotSimPose);
   }

  /** Reset pose history of the robot in the vision system simulation. */
   public void resetSimPose(Pose2d pose) {
       if (RobotBase.isSimulation()) visionSim.resetRobotPose(pose);
   }

  // /** A Field2d for visualizing our robot and objects on the field. */
   public Field2d getSimDebugField() {
       if (!RobotBase.isSimulation()) return null;
       return visionSim.getDebugField();
   }
  private boolean checkValidResult(List<PhotonTrackedTarget> result) {
    for (PhotonTrackedTarget target : result) {
      if (target.getFiducialId() > 8) {
        return false;
      }
    }
    return true;
  }
}
