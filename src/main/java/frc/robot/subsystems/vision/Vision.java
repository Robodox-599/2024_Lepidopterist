// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

// import static org.littletonrobotics.frc2024.RobotState.VisionObservation;
// import static org.littletonrobotics.frc2024.subsystems.apriltagvision.AprilTagVisionConstants.*;
// import static
// org.littletonrobotics.frc2024.subsystems.apriltagvision.AprilTagVisionIO.AprilTagVisionIOInputs;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.util.LoggedTunableNumber;
import java.util.*;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

/** Vision subsystem for AprilTag vision. */
public class Vision extends SubsystemBase {
  // // private static final LoggedTunableNumber timestampOffset =
  // //     new LoggedTunableNumber("AprilTagVision/TimestampOffset", -(1.0 / 50.0));

  // private final VisionIO io;
  // private final RobotState state; 
  // private final VisionIOAutoLogged inputs = new VisionIOinputsAutoLogged();
  // public Vision(VisionIO io, RobotState state){
  //   this.io = io;
  //   this.state = state;
    
  // }

  // @Override
  // public void periodic() {
  //   double timstamp = VisionConstants.getTimestampSeconds();

  //   io.readInput(inputs);
  //   Logger.processInputs("Vision", inputs);
  //   if (inputs.cameraSeesTarget()){

  //   } 
  // }

//     // Log robot poses
//     Logger.recordOutput("AprilTagVision/RobotPoses", allRobotPoses.toArray(Pose2d[]::new));
//     Logger.recordOutput("AprilTagVision/RobotPoses3d", allRobotPoses3d.toArray(Pose3d[]::new));
//     Logger.recordOutput("AprilTagVision/TagPoses", allTagPoses.toArray(Pose3d[]::new));
}
