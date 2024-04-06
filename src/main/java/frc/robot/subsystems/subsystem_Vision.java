// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.LimelightHelpers.LimelightResults;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.RobotConstants;

public class subsystem_Vision extends SubsystemBase {
  /** Creates a new subsystem_Vision. */
  private NetworkTable m_Table;
  // private NetworkTableEntry m_TV;
  // private double[] m_TY;
  // private double[] m_TX;
  // private double[] m_Yaw;
  // private double[] m_BotToTagVector;
  // private LimelightResults m_Results;
  private LimelightHelpers m_Helpers;
  private boolean m_HasTargets;
  private double m_Timestamp;
  // private double m_Pipeline;
  // private double m_LEDMode;
  // private AprilTagFieldLayout m_AprilTagField = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();;

  public subsystem_Vision() {
    m_Table = NetworkTableInstance.getDefault().getTable("limelight");
    m_Helpers = new LimelightHelpers();
    m_Table.getEntry("pipeline").setDouble(1.0);
    // m_TV = m_Table.getEntry("tv");
    m_HasTargets = false;
    m_Timestamp = 0.0;
  }

  public double[] getBotToTagVector(){
    return m_Table.getEntry("botpose").getDoubleArray(new double[1]);
  }

  public void setPipelineIndex(double index){
    m_Table.getEntry("pipeline").setDouble(index);
  }

  public double getPipelineIndex(){
    return m_Table.getEntry("getpipe").getDouble(0.0);
  }

  public void setLEDMode(double mode){
    m_Table.getEntry("ledMode").setDouble(mode);
  }

  public double getLEDMode(){
    return m_Table.getEntry("ledMode").getDouble(3.0);
  }

  public boolean hasTargets(){
    return m_HasTargets;
  }

  public LimelightHelpers getHelpers(){
    return m_Helpers;
  }

  public double getTimestampMS(){
    return m_Timestamp;
  }

  public Pose2d getEstimatedPose(){
    return RobotConstants.robotColor == Alliance.Blue ? 
            LimelightHelpers.getBotPose2d_wpiBlue("limelight") : 
            LimelightHelpers.getBotPose2d_wpiRed("limelight");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_HasTargets = m_Table.getEntry("tv").getDouble(0.0) == 1.0;
    if(m_HasTargets){
      m_Timestamp = m_Table.getEntry("tl").getDouble(0.0) + m_Table.getEntry("cl").getDouble(0.0);
    }
  }
}
