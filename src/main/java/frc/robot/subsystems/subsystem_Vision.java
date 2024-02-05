// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Retro;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

// import edu.wpi.first.math.geometry.Rotation3d;
//import com.revrobotics.CANSparkMax;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.units.Distance;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import java.util.Optional;
// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.simulation.PhotonCameraSim;
// import org.photonvision.simulation.SimCameraProperties;
// import org.photonvision.simulation.VisionSystemSim;
// import frc.robot.Robot;
// import frc.robot.Constants.SwerveConstants;
// import frc.robot.Constants.SwerveConstants;

public class subsystem_Vision extends SubsystemBase {
  /** Creates a new subsystem_Vision. */

  private PhotonCamera m_Camera;
  private boolean m_Detected;
  private PhotonPipelineResult m_Result;
  private double m_Distance;
  private double m_Yaw;
  // private Pose2d m_RobotPose;
  private AprilTagFieldLayout m_Field;
  private Transform3d m_RobotToCam;

  private NetworkTableInstance m_TableInstance;
  private NetworkTable m_Table;
  private double[] tx;
  private double[] ty;
  private NetworkTableEntry tv;
  private double[] tyaw;
  private LimelightResults m_LimelightResults;
  private boolean m_LLhasTargets;
  private LimelightHelpers m_LLresults;
  private double[] m_bot_pose;
  private double pipeline;
  private double ledMode;


  public subsystem_Vision() {
    m_TableInstance = NetworkTableInstance.getDefault();
    m_Table = m_TableInstance.getTable("limelight");
    
    //m_Camera = new PhotonCamera(VisionConstants.kCameraName);
    m_Camera = new PhotonCamera("Global_Shutter_Camera");
    m_Camera.setPipelineIndex(1);
    m_Field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    m_RobotToCam = VisionConstants.kRobotToCam;
    m_LLresults = new LimelightHelpers();
 
    m_Detected = false;
    m_LLhasTargets = false;
    m_Result = new PhotonPipelineResult();
    m_Distance = 0.0;
    m_Yaw = 0.0;
   
    tv = m_Table.getEntry("tv");
    
  }


 public double[] getTable(){
    m_bot_pose = m_Table
                .getEntry("botpose")
                .getDoubleArray(new double[1]);
  
    return m_bot_pose;
 }

public void setPipeline(float pipeline){
  m_Table.getEntry("pipeline").setDouble(pipeline);
}

public double getPipeline(){
  return pipeline;
}

public void setLEDMode(float ledMode){
  m_Table.getEntry("ledMode").setDouble(ledMode);
}

public double getLEDmode(){
  return ledMode;
}


  public boolean LLhasTargets(){
    return m_LLhasTargets;
  }

  public LimelightHelpers getLLResult(){
    return m_LLresults;
  }

  public Pose2d getLLRobotPose(){
    return LimelightHelpers.getBotPose2d("limelight");
  }

  public void smartdashboard(Field2d m_field) {
    SmartDashboard.putData("field", m_field);
  }

  @Override
  public void periodic() {
    pipeline = m_Table.getEntry("getpipe").getDouble(0);
    ledMode = m_Table.getEntry("ledMode").getDouble(3);

    // This method will be called once per scheduler run
    if(tv.getDouble(0) == 1.0){
    m_LLhasTargets = true;
  }
    
  

  }
}