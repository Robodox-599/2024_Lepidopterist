// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
// import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.configs.Pigeon2Configuration;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.units.Measure;
// import edu.wpi.first.units.Unit;
// import edu.wpi.first.units.Units;
// import edu.wpi.first.units.Velocity;
// import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SwerveModule;
import frc.robot.Constants.FrontLeftModule;
import frc.robot.Constants.FrontRightModule;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.BackLeftModule;
import frc.robot.Constants.BackRightModule;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.UnitConstants;
import frc.robot.Constants.SwerveConstants.DRIVE_STATE;
import frc.robot.Constants.SwerveConstants.Throttle;

//Climb Imports


public class subsystem_DriveTrain extends SubsystemBase {
  /** Creates a new subsystem_DriveTrain. */
  private Pigeon2 m_Gyro;
  private SwerveModule m_FrontLeft;
  private SwerveModule m_FrontRight;
  private SwerveModule m_BackLeft;
  private SwerveModule m_BackRight;
  
  private SwerveDriveKinematics m_Kinematics;
  private SwerveModulePosition[] m_ModulePositions;
  private SwerveDrivePoseEstimator m_PoseEstimator;
  
  private SwerveConstants.Throttle m_LinearThrottle;
  private SwerveConstants.Throttle m_AngularThrottle;

  private PIDController m_AutoOrientPID;
  
  private boolean m_IsPark;
  private double m_PreviousSetpoint = 0.0;
  private double m_NetBotAngle = 180.0;
  private int m_OrientCounter = 0;
  private Vector<N3> vec1 = VecBuilder.fill(0.7, 0.7, 0.1);
  private Vector<N3> vec2 = VecBuilder.fill(0.3, 0.3, 0.9);
  private SwerveConstants.DRIVE_STATE m_state = DRIVE_STATE.DRIVER_CONTROL;

  private Field2d m_Field;

  private ChassisSpeeds currentSpeeds;
  private Field2d m_field = new Field2d();

  public subsystem_DriveTrain() {
    m_Gyro = new Pigeon2(SwerveConstants.gyroID, SwerveConstants.CANBus);
    m_FrontLeft = new SwerveModule(FrontLeftModule.constants);
    m_FrontRight = new SwerveModule(FrontRightModule.constants);
    m_BackLeft = new SwerveModule(BackLeftModule.constants);
    m_BackRight = new SwerveModule(BackRightModule.constants);
    m_Kinematics = new SwerveDriveKinematics(SwerveConstants.frontLeft, 
                                            SwerveConstants.frontRight,
                                            SwerveConstants.backLeft,
                                            SwerveConstants.backRight);
    
    

    m_ModulePositions = new SwerveModulePosition[4];
    updateModulePositions();

    m_Field = new Field2d();
    
    m_PoseEstimator = new SwerveDrivePoseEstimator(m_Kinematics, 
                                                  new Rotation2d(),
                                                  m_ModulePositions,
                                                  new Pose2d(0.0, 0.0, 
                                                  new Rotation2d()
                                                  ), vec1, vec2);

    m_LinearThrottle = Throttle.LINEAR;
    m_AngularThrottle = Throttle.LINEAR;

    m_IsPark = false;
    m_AutoOrientPID = new PIDController(0.05, 0.0, 0.001);

    // m_Constraints = new TrapezoidProfile.Constraints(AutoConstants.MaxAngularSpeedMetersPerSecond, 
    //                                                 AutoConstants.MaxAngularAccelMetersPerSecondSquared);
    
    Timer.delay(1.0);
    resetModulesToAbsolute();
    m_Gyro.getConfigurator().apply(new Pigeon2Configuration());
    zeroGyro();

    m_AutoOrientPID.enableContinuousInput(-180.0, 180.0);
    m_AutoOrientPID.setTolerance(1.0);

    currentSpeeds = new ChassisSpeeds();

    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetOdometry,
      this::getRobotSpeeds,
      this::driveRobotRelative,
      // new HolonomicPathFollowerConfig(
      //   AutoConstants.MaxSpeedMetersPerSecond,
      //   Math.hypot(10.375, 10.375),
      //   new ReplanningConfig()),
      new HolonomicPathFollowerConfig(
        new PIDConstants(14.0, 0.0, 0.0),
        new PIDConstants(5.0, 0.0, 0.0),
        AutoConstants.MaxSpeedMetersPerSecond,
        Math.hypot(10.375, 10.375),
        new ReplanningConfig()
      ),
      () -> {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
      }, this);
  }

  public void updateModulePositions(){
    m_ModulePositions[0] = m_FrontLeft.getPosition();
    m_ModulePositions[1] = m_FrontRight.getPosition();
    m_ModulePositions[2] = m_BackLeft.getPosition();
    m_ModulePositions[3] = m_BackRight.getPosition();
  }

  public void swerveDrive(double xSpeedMetersPerSecond, 
                          double ySpeedMetersPerSecond, 
                          double zRotRadiansPerSecond, 
                          boolean isFieldRelative, 
                          boolean isOpenLoop){
    
  SmartDashboard.putNumber("before transform", zRotRadiansPerSecond);
  zRotRadiansPerSecond = transformZRot(zRotRadiansPerSecond);
  SmartDashboard.putNumber("after transform", zRotRadiansPerSecond);

    if(m_IsPark){
      xSpeedMetersPerSecond = 0.03 * Math.cos(25.0 * SwerveConstants.DEG_TO_RAD);
      ySpeedMetersPerSecond = 0.03 * Math.sin(25.0 * SwerveConstants.DEG_TO_RAD);
      zRotRadiansPerSecond = 0.0;
    }

    SmartDashboard.putNumber("X Speed", xSpeedMetersPerSecond);
    SmartDashboard.putNumber("Y Speed", ySpeedMetersPerSecond);
    SmartDashboard.putNumber("Z Rot", zRotRadiansPerSecond);
    
    setRobotSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                            xSpeedMetersPerSecond,
                            ySpeedMetersPerSecond,
                            zRotRadiansPerSecond,
                            getPoseYaw()));

    SwerveModuleState[] moduleStates = SwerveConstants.kinematics.toSwerveModuleStates(getRobotSpeeds());
    
    double maxVelocity = SwerveConstants.maxSpeed * 4.0;    
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxVelocity);

    SwerveModuleState frontLeft = moduleStates[0];
    SwerveModuleState frontRight = moduleStates[1];
    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backRight = moduleStates[3];
    
    // SmartDashboard.putNumber("FL Desired Angle", frontLeft.angle.getDegrees());
    // SmartDashboard.putNumber("FR Desired Angle", frontRight.angle.getDegrees());
    // SmartDashboard.putNumber("BL Desired Angle", backLeft.angle.getDegrees());
    // SmartDashboard.putNumber("BR Desired Angle", backRight.angle.getDegrees());
    // SmartDashboard.putNumber("FL Desired Speed", frontLeft.speedMetersPerSecond);
    // SmartDashboard.putNumber("FR Desired Speed", frontRight.speedMetersPerSecond);
    // SmartDashboard.putNumber("BL Desired Speed", backLeft.speedMetersPerSecond);
    // SmartDashboard.putNumber("BR Desired Speed", backRight.speedMetersPerSecond);
    
    m_FrontLeft.setDesiredState(frontLeft, isOpenLoop);
    m_FrontRight.setDesiredState(frontRight, isOpenLoop);
    m_BackLeft.setDesiredState(backLeft, isOpenLoop);
    m_BackRight.setDesiredState(backRight, isOpenLoop);

  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, AutoConstants.MaxSpeedMetersPerSecond);
    m_FrontLeft.setDesiredState(desiredStates[0], false);
    m_FrontRight.setDesiredState(desiredStates[1], false);
    m_BackLeft.setDesiredState(desiredStates[2], false);
    m_BackRight.setDesiredState(desiredStates[3], false);
  }

  public void changeLinearThrottle(){
    m_LinearThrottle = m_LinearThrottle == Throttle.LINEAR ? Throttle.NONLINEAR : Throttle.LINEAR;
  }
  
  public void changeAngularThrottle(){
    m_AngularThrottle = m_AngularThrottle == Throttle.LINEAR ? Throttle.NONLINEAR : Throttle.LINEAR;
  }

  public double setLinearThrottle(double input){
    // SmartDashboard.putString("Throttle Type", m_LinearThrottle.toString());
    return m_LinearThrottle == Throttle.LINEAR ? input : Math.signum(input) * (1.01 * Math.pow(input, 2) - 0.0202 * input + 0.0101);
  }

  public double setAngularThrottle(double input){
    return m_AngularThrottle == Throttle.LINEAR ? input : Math.signum(input) * (1.01 * Math.pow(input, 2) - 0.0202 * input + 0.0101);
  }

  public InstantCommand toggleLinearThrottleCommand(){
    return new InstantCommand(() -> changeLinearThrottle(), this);
  }

  public InstantCommand toggleAngularThrottleCommand(){
    return new InstantCommand(() -> changeAngularThrottle(), this);
  }

  public void zeroGyro(){
    m_Gyro.setYaw(0.0);
  }

  public InstantCommand zeroGyroInstantCommand(){
    return new InstantCommand(() -> zeroGyro(), this);
  }

  public void setNetBotAngle(double angle){ // deg
    m_NetBotAngle = angle;
  }

  public InstantCommand setNetBotAngleCommand(DoubleSupplier angle){
    return new InstantCommand(() -> setNetBotAngle(angle.getAsDouble()), this);
  }

  public void resetOdometry(Pose2d pose){
    updateModulePositions();
    m_PoseEstimator.resetPosition(m_Gyro.getRotation2d(), m_ModulePositions, pose);
  }

  public Pose2d getPose(){
    return m_PoseEstimator.getEstimatedPosition();
  }

  public Rotation2d getYaw(){
    double yaw = SwerveConstants.invertGyro ? 360.0 - m_Gyro.getYaw().getValueAsDouble() : m_Gyro.getYaw().getValueAsDouble();
    return Rotation2d.fromDegrees(yaw);
  }

  public Rotation2d getPoseYaw(){
    return m_PoseEstimator.getEstimatedPosition().getRotation();
  }

  public void resetModulesToAbsolute(){
    m_FrontLeft.resetToAbsolute();
    m_FrontRight.resetToAbsolute();
    m_BackLeft.resetToAbsolute();
    m_BackRight.resetToAbsolute();
  }

  public SwerveConstants.DRIVE_STATE getDriveState(){
    return m_state;
  }

   public void setDriveState(SwerveConstants.DRIVE_STATE newState){
    if (!DriverStation.isTeleopEnabled()){
      return;
    }
    switch (newState) {
      case ALIGNING_TO_DPAD:
        SmartDashboard.putString("State", "Aligning to DPAD");
        break;
      case SHOOTER_PREP:
        SmartDashboard.putString("State", "Shooter Prep");
        break;
      case DRIVER_CONTROL:
        SmartDashboard.putString("State", "Driver Control");
      default:
        SmartDashboard.putBoolean("STATE ERROR", true);
        break;
    }
    m_state = newState;
  }

  public Command setDriveStateCommand(SwerveConstants.DRIVE_STATE newState){
    return this.runOnce(() -> setDriveState(newState));
  }

  public InstantCommand setDriveStateInstantCommand(SwerveConstants.DRIVE_STATE newState){
    return new InstantCommand(() -> setDriveState(newState), this);
  }

  public void setDesiredAngle(double desiredAngle){
    if (!DriverStation.isTeleopEnabled()){
      return;
    }
    SmartDashboard.putNumber("desired align angle", desiredAngle);
    m_AutoOrientPID.setSetpoint(desiredAngle);
  }

  public double transformZRot(double oldZRot){
    if (m_PreviousSetpoint != m_AutoOrientPID.getSetpoint()){
      m_OrientCounter = 0;
    }
    m_PreviousSetpoint = m_AutoOrientPID.getSetpoint();

    if (oldZRot != 0 || m_state == DRIVE_STATE.DRIVER_CONTROL){ //note deadband already accounted for earlier
      m_state = DRIVE_STATE.DRIVER_CONTROL;
      return oldZRot;
    }
    
    if(m_AutoOrientPID.atSetpoint()){
      m_OrientCounter++; //wait 4 cycles before stopping
      if(m_OrientCounter >= 4){
        if (m_state == DRIVE_STATE.ALIGNING_TO_DPAD){
          m_state = DRIVE_STATE.DRIVER_CONTROL;
        }
        return 0; //return 0 if dpadding or at shooter pose
      }
    }
    return m_AutoOrientPID.calculate(m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees()); 
  }

  public boolean readyToShoot(){
    return Math.abs(m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees()
                    - m_AutoOrientPID.getSetpoint()) < SwerveConstants.readyToShootThreshold;
  }
  public void implementVisionPose(Pose2d visionPose, double timestamp){
    m_PoseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  public double getRoll(){
    return m_Gyro.getRoll().getValueAsDouble();
  }

  public void print(int module){
    switch(module){
      case 0:
        SmartDashboard.putNumber("FL Angle", m_FrontLeft.getState().angle.getDegrees());
        // SmartDashboard.putNumber("FL CANcoder w/o Offset", m_FrontLeft.getCANCoder().getDegrees());
        SmartDashboard.putNumber("FL Speed", m_FrontLeft.getState().speedMetersPerSecond);
        // SmartDashboard.putNumber("FL Error Units: " + m_FrontLeft.getErrorCodeUnits(), m_FrontLeft.getErrorCodeVal());
        break;
      
      case 1:
        SmartDashboard.putNumber("FR Angle", m_FrontRight.getState().angle.getDegrees());
        // SmartDashboard.putNumber("FR CANcoder w/o Offset", m_FrontRight.getCANCoder().getDegrees());
        SmartDashboard.putNumber("FR Speed", m_FrontRight.getState().speedMetersPerSecond);
        // SmartDashboard.putNumber("FR Error Units: " + m_FrontRight.getErrorCodeUnits(), m_FrontRight.getErrorCodeVal());
        break;
      
      case 2:
        SmartDashboard.putNumber("BL Angle", m_BackLeft.getState().angle.getDegrees());
        // SmartDashboard.putNumber("BL CANcoder w/o Offset", m_BackLeft.getCANCoder().getDegrees());
        SmartDashboard.putNumber("BL Speed", m_BackLeft.getState().speedMetersPerSecond);
        // SmartDashboard.putNumber("BL Error Units: " + m_BackLeft.getErrorCodeUnits(), m_BackLeft.getErrorCodeVal());
        break;
      
      case 3:
        SmartDashboard.putNumber("BR Angle", m_BackRight.getState().angle.getDegrees());
        // SmartDashboard.putNumber("BR CANcoder w/o Offset", m_BackRight.getCANCoder().getDegrees());
        SmartDashboard.putNumber("BR Speed", m_BackRight.getState().speedMetersPerSecond);
        // SmartDashboard.putNumber("BR Error Units: " + m_BackRight.getErrorCodeUnits(), m_BackRight.getErrorCodeVal());
        break;
      
      default:
        break;
    }
  }

  public void setChoreoTrajectory(ChoreoTrajectory trajectory){
    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.MaxSpeedMetersPerSecond, 
                                                  AutoConstants.MaxAccelMetersPerSecondSquared);
    Trajectory traj = TrajectoryGenerator.generateTrajectory(Arrays.asList(trajectory.getPoses()), config);
    m_Field.getObject("traj").setTrajectory(traj);
  }

  public void setPathPlannerTrajectory(PathPlannerPath trajectory){
    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.MaxSpeedMetersPerSecond, 
                                                  AutoConstants.MaxAccelMetersPerSecondSquared);
    Trajectory traj = TrajectoryGenerator.generateTrajectory(trajectory.getPathPoses(), config);
    m_Field.getObject("traj").setTrajectory(traj);
  }

  public void setRobotSpeeds(ChassisSpeeds speeds){
    currentSpeeds = speeds;
  }

  public ChassisSpeeds getRobotSpeeds(){
    return currentSpeeds;
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    SwerveModuleState[] moduleStates = SwerveConstants.kinematics.toSwerveModuleStates(
                                                      ChassisSpeeds.fromFieldRelativeSpeeds(
                                                        speeds, getPoseYaw()));
    setModuleStates(moduleStates);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (m_state) {
      case ALIGNING_TO_DPAD:
        SmartDashboard.putString("State", "Aligning to DPAD");
        break;
      case SHOOTER_PREP:
        SmartDashboard.putString("State", "Shooter Prep");
        break;
      case DRIVER_CONTROL:
        SmartDashboard.putString("State", "Driver Control");
        break;
      default:
        SmartDashboard.putBoolean("STATE ERROR", true);
        break;
    }
    // m_PoseEstimator.update(m_Gyro.getRotation2d(), m_ModulePositions);
    double deltaTheta = m_NetBotAngle >= 0 ? m_NetBotAngle : m_NetBotAngle + 360.0;
    m_PoseEstimator.update(m_Gyro.getRotation2d().plus(Rotation2d.fromDegrees(deltaTheta)), m_ModulePositions);
    SmartDashboard.putNumber("Est X Position", m_PoseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Est Y Position", m_PoseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Est Pose Yaw", m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees());
    SmartDashboard.putBoolean("SLOW DRIVE MODE", m_LinearThrottle == Throttle.LINEAR);
    SmartDashboard.putBoolean("SLOW ROT MODE", m_AngularThrottle == Throttle.LINEAR);
    updateModulePositions();
    double absAngle = m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees();
    absAngle = absAngle >= 0 ? absAngle : absAngle + 360.0;
    // m_field.setRobotPose(m_PoseEstimator.getEstimatedPosition().div(1/UnitConstants.FETRES_TO_MEET));
    Pose2d scaledPos = m_PoseEstimator.getEstimatedPosition().div(UnitConstants.METRES_TO_FEETRES);
    m_field.setRobotPose(scaledPos.getX(), scaledPos.getY(), Rotation2d.fromDegrees(absAngle));
    SmartDashboard.putData("Robot Position", m_field);
    SmartDashboard.putNumber("offset", m_NetBotAngle);
  }
}
