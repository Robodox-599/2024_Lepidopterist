// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.Pigeon2;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SwerveModule;
import frc.robot.Constants.DPAD;
import frc.robot.Constants.FrontLeftModule;
import frc.robot.Constants.FrontRightModule;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.BackLeftModule;
import frc.robot.Constants.BackRightModule;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.DPAD.ORIENTATION;
import frc.robot.Constants.SwerveConstants.Throttle;

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
  
  private SwerveConstants.Throttle m_Throttle;

  private PIDController m_AutoOrientPID;
  
  private boolean m_IsAutoOrient;
  private boolean m_IsPark;
  
  private int m_DPAD;
  private int m_OrientCounter;

  private Vector<N3> vec1 = VecBuilder.fill(0.7, 0.7, 0.1);
  private Vector<N3> vec2 = VecBuilder.fill(0.3, 0.3, 0.9);
  
  public subsystem_DriveTrain() {
    m_Gyro = new Pigeon2(SwerveConstants.canCoderID, "DriveCANivore");
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
    
    m_PoseEstimator = new SwerveDrivePoseEstimator(m_Kinematics, 
                                                  new Rotation2d(),
                                                  m_ModulePositions,
                                                  new Pose2d(0.0, 0.0, 
                                                  new Rotation2d()
                                                  ), vec1, vec2);

    m_Throttle = Throttle.LINEAR;

    m_IsAutoOrient = false;
    m_IsPark = false;
    m_OrientCounter = 0;
    m_DPAD = DPAD.value(ORIENTATION.NON_ORIENTED);
    m_AutoOrientPID = new PIDController(0.05, 0.0, 0.001);
    // m_Constraints = new TrapezoidProfile.Constraints(AutoConstants.MaxAngularSpeedMetersPerSecond, 
    //                                                 AutoConstants.MaxAngularAccelMetersPerSecondSquared);
    
    Timer.delay(1.0);
    resetModulesToAbsolute();
    m_Gyro.getConfigurator().apply(new Pigeon2Configuration());
    zeroGyro();

    m_AutoOrientPID.enableContinuousInput(-180.0, 180.0);
    m_AutoOrientPID.setTolerance(1.0);
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
    
    zRotRadiansPerSecond = m_IsAutoOrient ? getAngularVelocity() : zRotRadiansPerSecond;
    if(m_IsPark){
      xSpeedMetersPerSecond = 0.03 * Math.cos(25.0 * SwerveConstants.DEG_TO_RAD);
      ySpeedMetersPerSecond = 0.03 * Math.sin(25.0 * SwerveConstants.DEG_TO_RAD);
      zRotRadiansPerSecond = 0.0;
    }
    // SmartDashboard.putNumber("X Speed", xSpeedMetersPerSecond);
    // SmartDashboard.putNumber("Y Speed", ySpeedMetersPerSecond);
    // SmartDashboard.putNumber("Z Rot", zRotRadiansPerSecond);
    
    SwerveModuleState[] moduleStates = SwerveConstants.kinematics.toSwerveModuleStates(
      ChassisSpeeds.fromFieldRelativeSpeeds(
                            xSpeedMetersPerSecond,
                            ySpeedMetersPerSecond,
                            zRotRadiansPerSecond,
                            getPoseYaw()));

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.maxSpeed * 4.0);
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

  public void changeThrottle(){
    // if(m_Throttle == Throttle.LINEAR){
    //   m_Throttle = Throttle.NONLINEAR;
    //   SmartDashboard.putBoolean("SLOW MODE", false);
    // } 
    // if(m_Throttle == Throttle.NONLINEAR) {
    //   m_Throttle = Throttle.LINEAR;
    //   SmartDashboard.putBoolean("SLOW MODE", true);
    // }
    m_Throttle = m_Throttle == Throttle.LINEAR ? Throttle.NONLINEAR : Throttle.LINEAR;
  }

  public double setThrottle(double input){
    // SmartDashboard.putString("Throttle Type", m_Throttle.toString());
    return m_Throttle == Throttle.LINEAR ? input : Math.signum(input) * (1.01 * Math.pow(input, 2) - 0.0202 * input + 0.0101);
  }

  // public Command toggleThrottleCommand(){
  //   return this.runOnce(() -> changeThrottle());
  // }

  public InstantCommand toggleThrottleInstantCommand(){
    return new InstantCommand(() -> changeThrottle(), this);
  }

  public void zeroGyro(){
    m_Gyro.setYaw(0.0);
  }

  // public Command zeroGyroCommand(){
  //   return this.runOnce(() -> zeroGyro());
  // }

  public InstantCommand zeroGyroInstantCommand(){
    return new InstantCommand(() -> zeroGyro(), this);
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

  public void setAutoOrient(boolean isOrientFront, boolean isOrientBack, double rotVelocity){
    if((Math.abs(rotVelocity) <= ControllerConstants.deadband) && DriverStation.isTeleopEnabled()){
      if(isOrientFront){
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
          m_DPAD = DPAD.value(ORIENTATION.RIGHT);
        } else {
          m_DPAD = DPAD.value(ORIENTATION.DOWN);
          m_IsAutoOrient = true;
        }
      }
    } else {
      m_IsAutoOrient = false;
      m_DPAD = DPAD.value(ORIENTATION.NON_ORIENTED);
    }
  }

  public double getAngularVelocity(){
    m_AutoOrientPID.setSetpoint(m_DPAD);
    if(m_AutoOrientPID.atSetpoint()){
      m_OrientCounter++;
      if(m_OrientCounter >= 4){
        m_IsAutoOrient = false;
        m_DPAD = DPAD.value(ORIENTATION.NON_ORIENTED);
        m_OrientCounter = 0;
        return 0.0;
      }
    }
    return m_AutoOrientPID.calculate(m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees());
  }

  public void print(int module){
    switch(module){
      case 0:
        // SmartDashboard.putNumber("FL Angle", m_FrontLeft.getState().angle.getDegrees());
        // SmartDashboard.putNumber("FL CANcoder w/o Offset", m_FrontLeft.getCANCoder().getDegrees());
        // SmartDashboard.putNumber("FL Speed", m_FrontLeft.getState().speedMetersPerSecond);
        // SmartDashboard.putNumber("FL Error Units: " + m_FrontLeft.getErrorCodeUnits(), m_FrontLeft.getErrorCodeVal());
        break;
      
      case 1:
        // SmartDashboard.putNumber("FR Angle", m_FrontRight.getState().angle.getDegrees());
        // SmartDashboard.putNumber("FR CANcoder w/o Offset", m_FrontRight.getCANCoder().getDegrees());
        // SmartDashboard.putNumber("FR Speed", m_FrontRight.getState().speedMetersPerSecond);
        // SmartDashboard.putNumber("FR Error Units: " + m_FrontRight.getErrorCodeUnits(), m_FrontRight.getErrorCodeVal());
        break;
      
      case 2:
        // SmartDashboard.putNumber("BL Angle", m_BackLeft.getState().angle.getDegrees());
        // SmartDashboard.putNumber("BL CANcoder w/o Offset", m_BackLeft.getCANCoder().getDegrees());
        // SmartDashboard.putNumber("BL Speed", m_BackLeft.getState().speedMetersPerSecond);
        // SmartDashboard.putNumber("BL Error Units: " + m_BackLeft.getErrorCodeUnits(), m_BackLeft.getErrorCodeVal());
        break;
      
      case 3:
        // SmartDashboard.putNumber("BR Angle", m_BackRight.getState().angle.getDegrees());
        // SmartDashboard.putNumber("BR CANcoder w/o Offset", m_BackRight.getCANCoder().getDegrees());
        // SmartDashboard.putNumber("BR Speed", m_BackRight.getState().speedMetersPerSecond);
        // SmartDashboard.putNumber("BR Error Units: " + m_BackRight.getErrorCodeUnits(), m_BackRight.getErrorCodeVal());
        break;
      
      default:
        break;
    }
  }

  public void implementVisionPose(double[] measurements){
    if(measurements.length == 4){
      SmartDashboard.putNumber("VisXPos", measurements[0]);
      SmartDashboard.putNumber("VisYPos", measurements[1]);
      SmartDashboard.putNumber("VisRotDeg", measurements[2]);
      if((Math.abs(measurements[0] - getPose().getX()) < 1.0 && Math.abs(measurements[1] - getPose().getY()) < 1.0) || DriverStation.isDisabled()){
        m_PoseEstimator.addVisionMeasurement(new Pose2d(measurements[0], measurements[1], Rotation2d.fromDegrees(measurements[2])), measurements[3]);
      }
    }
  }

  public void togglePark(){
    m_IsPark = !m_IsPark;
  }

  public InstantCommand toggleParkCommand(){
    return new InstantCommand(() -> togglePark(), this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Est X Position", m_PoseEstimator.getEstimatedPosition().getX());
    // SmartDashboard.putNumber("Est Y Position", m_PoseEstimator.getEstimatedPosition().getY());
    // SmartDashboard.putNumber("Est Pose Yaw", m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees());
    // SmartDashboard.putBoolean("SLOW MODE", m_Throttle == Throttle.LINEAR);
    print(0);
    print(1);
    print(2);
    print(3);
    m_PoseEstimator.update(m_Gyro.getRotation2d(), m_ModulePositions);
    updateModulePositions();
  }
}
