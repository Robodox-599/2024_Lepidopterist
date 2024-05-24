package frc.robot.subsystems.vision;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

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
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

public class VisionIOLimelight implements VisionIO {
    NetworkTable tableA = NetworkTableInstance.getDefault().getTable("limelight");
    RobotState RobotState;
    AtomicReference<VisionIOinput> inputlatest = new AtomicReference<>(new VisionIOinput());
    public VisionIOLimelight(RobotState RobotState){
        this.RobotState = RobotState;
        setLLSettings();
    }
    private void setLLSettings(){
        double[] CameraPosition = {
            0, // camera x pose in meters 
            0, // camera y pose in meters
            VisionConstants.cameraHeight, // camera z pose in meters
            0, // camera roll in degrees
            VisionConstants.cameraPitch, // camera pitch in degrees
            0 // camera yaw in degrees
        };
        var gyroAngle = field
    }
}
