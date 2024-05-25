// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

// import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOinput {
    public double x;
    public double y;
    public double rotation;
    public double timestamp;
    public double maxAmbiguity;
    public double maxDistance;
    public double minDistance;
    public Pose2d estimation = new Pose2d();
    public boolean hasTarget = false;
    public boolean isNew; // is new pose
    public int tagCount = 0;
    public double timestamps = 0;
    public int singleIDUsed;
    public double singleIDUsedDouble;
  }
  // void readInput(VisionIOinput inputs);
  // void pollNetworkTables();
  public default void updateInputs(VisionIOinput inputs) {}

  default String getName() {
    return "";
  }

  default void setReferencePose(Pose2d pose) {}
}
