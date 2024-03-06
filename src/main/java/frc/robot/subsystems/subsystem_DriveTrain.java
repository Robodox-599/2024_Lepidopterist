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
import edu.wpi.first.wpilibj2.command.Command;
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

  private PIDController m_PitchCorrectionPID;
  private PIDController m_RollCorrectionPID;
  private PIDController m_AutoOrientPID;
  
  private boolean m_IsBalancing;
  private boolean m_IsAutoOrient;
  private boolean m_isAiming;
  // private boolean m_IsPark;
  
  private int m_DPAD;
  private int m_OrientCounter;

  private Vector<N3> vec1 = VecBuilder.fill(0.7, 0.7, 0.1);
  private Vector<N3> vec2 = VecBuilder.fill(0.3, 0.3, 0.9);
  
  public subsystem_DriveTrain() {
    m_Gyro = new Pigeon2(SwerveConstants.canCoderID, "LunaDriveCANivore");
    m_FrontLeft = new SwerveModule(FrontLeftModule.constants);
    m_FrontRight = new SwerveModule(FrontRightModule.constants);
    m_BackLeft = new SwerveModule(BackLeftModule.constants);
    m_BackRight = new SwerveModule(BackRightModule.constants);
    m_Kinematics = new SwerveDriveKinematics(SwerveConstants.frontLeft, 
                                            SwerveConstants.frontRight,
                                            SwerveConstants.backLeft,
                                            SwerveConstants.backRight);
    // m_FrontLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)), false);
    // m_FrontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)), false);
    // m_BackLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)), false);
    // m_BackRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)), false);    

    m_ModulePositions = new SwerveModulePosition[4];
    updateModulePositions();
    
    m_PoseEstimator = new SwerveDrivePoseEstimator(m_Kinematics, 
                                                  // Rotation2d.fromDegrees(0.0),
                                                  new Rotation2d(),
                                                  m_ModulePositions,
                                                  new Pose2d(0.0, 0.0, 
                                                  // Rotation2d.fromDegrees(0.0)
                                                  new Rotation2d()
                                                  ), vec1, vec2);
    m_PitchCorrectionPID = new PIDController(0.0, 0.0, 0.0);
    m_RollCorrectionPID = new PIDController(0.01, 0.0, 0.0);

    m_Throttle = Throttle.LINEAR;
    m_isAiming = false;
    m_IsBalancing = false;
    m_IsAutoOrient = false;
    // m_IsPark = false;
    m_OrientCounter = 0;
    m_DPAD = DPAD.value(ORIENTATION.NON_ORIENTED);
    m_AutoOrientPID = new PIDController(0.05, 0.0, 0.001);
    // m_Constraints = new TrapezoidProfile.Constraints(AutoConstants.MaxAngularSpeedMetersPerSecond, 
    //                                                 AutoConstants.MaxAngularAccelMetersPerSecondSquared);
    
    Timer.delay(1.0);
    // resetModulesToAbsolute();
    // zeroModules();
    m_Gyro.getConfigurator().apply(new Pigeon2Configuration());
    zeroGyro();

    m_PitchCorrectionPID.setSetpoint(0.0);
    m_PitchCorrectionPID.setTolerance(2.0);

    m_RollCorrectionPID.setSetpoint(0.0);
    m_RollCorrectionPID.setTolerance(2.0);

    m_AutoOrientPID.enableContinuousInput(-180.0, 180.0);
    m_AutoOrientPID.setTolerance(1.0);
  }

  public void updateModulePositions(){
    m_ModulePositions[0] = m_FrontLeft.getPosition();
    m_ModulePositions[1] = m_FrontRight.getPosition();
    m_ModulePositions[2] = m_BackLeft.getPosition();
    m_ModulePositions[3] = m_BackRight.getPosition();
  }

  public boolean isBalanced(){
    return Math.abs(m_Gyro.getPitch().getValueAsDouble()) < 2 && Math.abs(m_Gyro.getRoll().getValueAsDouble()) < 2;
  }

  public void swerveDrive(double xSpeedMetersPerSecond, 
                          double ySpeedMetersPerSecond, 
                          double zRotRadiansPerSecond, 
                          boolean isFieldRelative, 
                          boolean isOpenLoop){
    if (m_IsBalancing){
      xSpeedMetersPerSecond += addRollCorrection() * getPoseYaw().getCos();
    }
    zRotRadiansPerSecond = m_IsAutoOrient ? getAngularVelocity() : zRotRadiansPerSecond;

    SwerveModuleState[] moduleStates = SwerveConstants.kinematics.toSwerveModuleStates(
      // isFieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
      //                       xSpeedMetersPerSecond,
      //                       ySpeedMetersPerSecond,
      //                       zRotRadiansPerSecond,
      //                       getPoseYaw()
      //                       ) : 
      //                   new ChassisSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, zRotRadiansPerSecond)
      ChassisSpeeds.fromFieldRelativeSpeeds(
                            xSpeedMetersPerSecond,
                            ySpeedMetersPerSecond,
                            zRotRadiansPerSecond,
                            getPoseYaw()
                            )
    );
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.maxSpeed);
    SwerveModuleState frontLeft = moduleStates[0];
    SwerveModuleState frontRight = moduleStates[1];
    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backRight = moduleStates[3];

    m_FrontLeft.setDesiredState(frontLeft, isOpenLoop);
    m_FrontRight.setDesiredState(frontRight, isOpenLoop);
    m_BackLeft.setDesiredState(backLeft, isOpenLoop);
    m_BackRight.setDesiredState(backRight, isOpenLoop);

    SmartDashboard.putNumber("FL Desired Angle", frontLeft.angle.getDegrees());
    SmartDashboard.putNumber("FR Desired Angle", frontRight.angle.getDegrees());
    SmartDashboard.putNumber("BL Desired Angle", backLeft.angle.getDegrees());
    SmartDashboard.putNumber("BR Desired Angle", backRight.angle.getDegrees());
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, AutoConstants.MaxSpeedMetersPerSecond);
    m_FrontLeft.setDesiredState(desiredStates[0], false);
    m_FrontRight.setDesiredState(desiredStates[1], false);
    m_BackLeft.setDesiredState(desiredStates[2], false);
    m_BackRight.setDesiredState(desiredStates[3], false);
  }

  public double setThrottle(double input){
    return m_Throttle != Throttle.NONLINEAR ? input : Math.signum(input) * (1.01 * Math.pow(input, 2) - 0.0202 * input + 0.0101);
  }

  public void changeThrottle(){
    SmartDashboard.putBoolean("Signal Received", false);
    if(m_Throttle == Throttle.LINEAR){
      m_Throttle = Throttle.NONLINEAR;
      SmartDashboard.putBoolean("DRIVE MODE", false);
    } else {
      m_Throttle = Throttle.NONLINEAR;
      SmartDashboard.putBoolean("DRIVE MODE", true);
    }
  }

  public Command toggleThrottleCommand(){
    return this.runOnce(() -> changeThrottle());
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

  public void toggleBalanceCorrection(){
    m_IsBalancing = !m_IsBalancing;
  }

  public void enableBalanceCorrection(){
    m_IsBalancing = true;
  }

  public void disableBalanceCorrection(){
    m_IsBalancing = false;
  }
  public void toggleYawCorrection(){
    m_isAiming = !m_isAiming;
  }

  public void enableYawCorrection(){
    m_isAiming = true;
  }

  public void disableYawCorrection(){
    m_isAiming = false;
  }

  public void zeroGyro(){
    m_Gyro.setYaw(0.0);
  }

  public Command zeroGyroCommand(){
    return this.runOnce(() -> zeroGyro());
  }

  public void resetModulesToAbsolute(){
    m_FrontLeft.resetToAbsolute();
    m_FrontRight.resetToAbsolute();
    m_BackLeft.resetToAbsolute();
    m_BackRight.resetToAbsolute();
  }

  public void zeroModules(){
    m_FrontLeft.setDesiredAngle(Rotation2d.fromDegrees(0.0));
    m_FrontRight.setDesiredAngle(Rotation2d.fromDegrees(0.0));
    m_BackLeft.setDesiredAngle(Rotation2d.fromDegrees(0.0));
    m_BackRight.setDesiredAngle(Rotation2d.fromDegrees(0.0));
  }

  public double addPitchCorrection(){
    return m_PitchCorrectionPID.calculate(m_Gyro.getPitch().getValueAsDouble());
  }

  public double addRollCorrection(){
    return m_RollCorrectionPID.calculate(m_Gyro.getRoll().getValueAsDouble());
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
        SmartDashboard.putNumber("FL Angle", m_FrontLeft.getState().angle.getDegrees());
        SmartDashboard.putNumber("FL CANcoder w/o Offset", m_FrontLeft.getCANCoder().getDegrees());
        SmartDashboard.putNumber("FL CANcoder w/ Offset", m_FrontLeft.getValDegWithOffset());
        SmartDashboard.putNumber("FL Error Units: " + m_FrontLeft.getErrorCodeUnits(), m_FrontLeft.getErrorCodeVal());
        break;
      case 1:
        SmartDashboard.putNumber("FR Angle", m_FrontRight.getState().angle.getDegrees());
        SmartDashboard.putNumber("FR CANcoder w/o Offset", m_FrontRight.getCANCoder().getDegrees());
        SmartDashboard.putNumber("FR CANcoder w/ Offset", m_FrontRight.getValDegWithOffset());
        SmartDashboard.putNumber("FR Error Units: " + m_FrontRight.getErrorCodeUnits(), m_FrontRight.getErrorCodeVal());
        break;
      case 2:
        SmartDashboard.putNumber("BL Angle", m_BackLeft.getState().angle.getDegrees());
        SmartDashboard.putNumber("BL CANcoder w/o Offset", m_BackLeft.getCANCoder().getDegrees());
        SmartDashboard.putNumber("BL CANcoder w/ Offset", m_BackLeft.getValDegWithOffset());
        SmartDashboard.putNumber("BL Error Units: " + m_BackLeft.getErrorCodeUnits(), m_BackLeft.getErrorCodeVal());
        break;
      case 3:
        SmartDashboard.putNumber("BR Angle", m_BackRight.getState().angle.getDegrees());
        SmartDashboard.putNumber("BR CANcoder w/o Offset", m_BackRight.getCANCoder().getDegrees());
        SmartDashboard.putNumber("BR CANcoder w/ Offset", m_BackRight.getValDegWithOffset());
        SmartDashboard.putNumber("BR Error Units: " + m_BackRight.getErrorCodeUnits(), m_BackRight.getErrorCodeVal());
        break;
      default:
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Est X Position", m_PoseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Est Y Position", m_PoseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Est Pose Yaw", m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees());
    print(0);
    // SmartDashboard.putNumber("FL Angle", m_FrontLeft.getState().angle.getDegrees());
    // SmartDashboard.putNumber("BL Angle", m_BackLeft.getState().angle.getDegrees());
    // SmartDashboard.putNumber("BR Angle", m_BackRight.getState().angle.getDegrees());
    // SmartDashboard.putNumber("FL CANcoder", m_FrontLeft.getValDeg());
    // SmartDashboard.putNumber("BL CANcoder", m_BackLeft.getValDeg());
    // SmartDashboard.putNumber("BR CANcoder", m_BackRight.getValDeg());
    m_PoseEstimator.update(m_Gyro.getRotation2d(), m_ModulePositions);
  }
}