// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.configs.Pigeon2Configuration;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SwerveModule;
import frc.robot.Constants.FrontLeftModule;
import frc.robot.Constants.FrontRightModule;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.BackLeftModule;
import frc.robot.Constants.BackRightModule;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.UnitConstants;
import frc.robot.Constants.SwerveConstants.Throttle;
import frc.robot.Constants.SwerveConstants.DRIVE_STATE;
import frc.robot.Constants.SwerveConstants.DPAD;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

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

  private boolean m_ToggleGyro;
  
  private double m_PreviousSetpoint = 0.0;
  private int m_OrientCounter = 0;

  private Vector<N3> vec1 = VecBuilder.fill(0.7, 0.7, 0.1);
  private Vector<N3> vec2 = VecBuilder.fill(0.3, 0.3, 0.9);
  private SwerveConstants.DRIVE_STATE m_state = DRIVE_STATE.DRIVER_CONTROL;

  private Field2d m_Field;
  private ChassisSpeeds currentSpeeds;

  private Pose2d m_FinalPose = new Pose2d();

  Translation2d m_SpeakerCenter = new Translation2d();

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
    
    m_ToggleGyro =false;

    m_Field = new Field2d();
    m_AutoOrientPID = new PIDController(0.2, 0.0, 0.04);
    
    m_PoseEstimator = new SwerveDrivePoseEstimator(m_Kinematics, 
                                                  Rotation2d.fromDegrees(180.0),
                                                  m_ModulePositions,
                                                  new Pose2d(3.0, 5.0, 
                                                  new Rotation2d()
                                                  ), vec1, vec2);
    
    m_LinearThrottle = Throttle.NONLINEAR;
    m_AngularThrottle = Throttle.NONLINEAR;
    updateModulePositions();

    Timer.delay(1.0);
    resetModulesToAbsolute();

    m_Gyro.getConfigurator().apply(new Pigeon2Configuration());
    
    m_AutoOrientPID.enableContinuousInput(-180.0, 180.0);
    m_AutoOrientPID.setTolerance(1.0);
    
    currentSpeeds = new ChassisSpeeds();

    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetOdometry,
      this::getRobotSpeeds,
      this::driveRobotRelative,
      new HolonomicPathFollowerConfig(
        new PIDConstants(14.0, 0.0, 0.0),
        new PIDConstants(14.0, 0.0, 0.0),
        AutoConstants.MaxSpeedMetersPerSecond,
        Math.hypot(10.375, 10.375),
        new ReplanningConfig()
        ),
        () -> RobotConstants.robotColor == DriverStation.Alliance.Red, 
        this);

    SmartDashboard.putData("zero gyro", zeroGyroInstantCommand());
    zeroGyro();
  }

  public void updateModulePositions(){
    m_ModulePositions[0] = m_FrontLeft.getPosition();
    m_ModulePositions[1] = m_FrontRight.getPosition();
    m_ModulePositions[2] = m_BackLeft.getPosition();
    m_ModulePositions[3] = m_BackRight.getPosition();
  }

  public void park(){
    m_FrontLeft.setDesiredAngle(Rotation2d.fromDegrees(45.0));
    m_FrontRight.setDesiredAngle(Rotation2d.fromDegrees(-45.0));
    m_BackLeft.setDesiredAngle(Rotation2d.fromDegrees(45.0));
    m_BackRight.setDesiredAngle(Rotation2d.fromDegrees(-45.0));
  }

  public InstantCommand parkCommand(){
    return new InstantCommand(() -> park(), this);
  }

  public void toggleGyro(){
    m_ToggleGyro = !m_ToggleGyro;
  }

  public InstantCommand toggleGyroCommand(){
    return new InstantCommand(() -> toggleGyro(), this);
  }

  public boolean getGyroToggle(){
    return m_ToggleGyro;
  }

  public void swerveDrive(double xSpeedMetersPerSecond, 
                          double ySpeedMetersPerSecond, 
                          double zRotRadiansPerSecond, 
                          boolean isFieldRelative, 
                          boolean isOpenLoop){
    
    SmartDashboard.putNumber("before transform", zRotRadiansPerSecond);
    zRotRadiansPerSecond = transformZRot(zRotRadiansPerSecond);

    SmartDashboard.putNumber("X Speed", xSpeedMetersPerSecond);
    SmartDashboard.putNumber("Y Speed", ySpeedMetersPerSecond);
    SmartDashboard.putNumber("Z Rot Speed", zRotRadiansPerSecond);
    
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

  public void flipBot(){
    double angle = getYaw().getDegrees();
    angle = angle <= 0 ? angle + 180.0 : angle - 180.0;
    setDesiredAngle(angle);
  }

  public InstantCommand flipBotCommand(){
    return new InstantCommand(() -> flipBot(), this);
  }

  public void zeroGyro(){
    m_Gyro.setYaw(0);
    m_PoseEstimator.resetPosition(m_Gyro.getRotation2d(), m_ModulePositions, getPose());
  }

  public void invertGyro(){
    m_Gyro.setYaw(180.0);
  }

  public Command zeroGyroInstantCommand(){
    return new InstantCommand(() -> zeroGyro(), this).ignoringDisable(true);
  }

  public InstantCommand invertGyroInstantCommand(){
    return new InstantCommand(() -> invertGyro(), this);
  }

  public void resetOdometry(Pose2d pose){
    updateModulePositions();
    m_PoseEstimator.resetPosition(m_Gyro.getRotation2d(), m_ModulePositions, pose);
  }

  public InstantCommand resetSubwoofer(DoubleSupplier side){
    return new InstantCommand(() -> resetOdometry(
      new Pose2d(FieldConstants.subwooferPoses[
      RobotConstants.robotColor == Alliance.Blue ? 
                                  (int)side.getAsDouble() + 3 : 
                                  (int)side.getAsDouble()].getTranslation(), getPoseYaw())), this);   
  }

  // public InstantCommand resetSubwoofer(FieldConstants.SubwooferSide side){
  //   Translation2d resetPose = new Translation2d(); //im jumping
  //   boolean isBlue = RobotConstants.robotColor == Alliance.Blue;
  //   switch (side){
  //     case LEFT:
  //       resetPose = isBlue ? FieldConstants.blueSubwooferAmp : FieldConstants.redSubwooferSource;
  //     case CENTER:
  //       resetPose = isBlue ? FieldConstants.blueSubwooferCenter : FieldConstants.redSubwooferCenter;
  //     case RIGHT:
  //       resetPose = isBlue ? FieldConstants.blueSubwooferSource : FieldConstants.redSubwooferAmp;
  //   }
  //     return new InstantCommand(() -> resetOdometry(
  //       new Pose2d(resetPose, resetPose.getAngle())));
  // }

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

  // public Command setDriveStateCommand(SwerveConstants.DRIVE_STATE newState){
  //   return this.runOnce(() -> setDriveState(newState));
  // }

  // public InstantCommand setDriveStateInstantCommand(SwerveConstants.DRIVE_STATE newState){
  //   return new InstantCommand(() -> setDriveState(newState), this);
  // }
  public void DPAD(DPAD input){
    double desiredAngle = -360.0;
    switch(input){
      case UP:
        desiredAngle = 0;
        break;
      case UPRIGHT:
        desiredAngle = -45;
        break;
      case RIGHT:
        desiredAngle = -90;
        break;
      case DOWNRIGHT:
        desiredAngle = -135;
        break;
      case DOWN:
        desiredAngle = 180;
        break;
      case DOWNLEFT:
        desiredAngle = 135;
        break;
      case LEFT:
        desiredAngle = 90;
        break;
      case UPLEFT:
        desiredAngle = 45;
        break;
    }
    if (desiredAngle != -360){
      setDriveState(DRIVE_STATE.ALIGNING_TO_DPAD);
      if (RobotConstants.robotColor == Alliance.Red){
        setDesiredAngle(desiredAngle > 0 ? desiredAngle - 180: desiredAngle + 180);
      } else { //default blue 
        setDesiredAngle(desiredAngle);
      }
    }
  }  

  public Command DPADCommand(DPAD input){
    return new InstantCommand(() -> DPAD(input), this); // do require
  }

  public void setDesiredAngle(double desiredAngle){
    if (!DriverStation.isTeleopEnabled()){
      return;
    }
    SmartDashboard.putNumber("desired align angle", desiredAngle);
    m_AutoOrientPID.setSetpoint(desiredAngle);
  }

  public void autoYaw(){
    // Translation2d m_speakerCenter = RobotConstants.robotColor == Alliance.Blue ? 
    //                                 FieldConstants.blueSpeakerCenter.toTranslation2d() :
    //                                 FieldConstants.redSpeakerCenter.toTranslation2d();
    // Translation2d drivePose = getPose().getTranslation();
    // SmartDashboard.putNumber("fieldDist", m_speakerCenter.getDistance(drivePose));
    // if(m_state == DRIVE_STATE.SHOOTER_PREP){
    Translation2d delta = m_SpeakerCenter.minus(getPose().getTranslation());
    double desiredAngleDeg = Math.atan2(delta.getY(), delta.getX()) * UnitConstants.RAD_TO_DEG;
    SmartDashboard.putNumber("deltaY", delta.getY());
    SmartDashboard.putNumber("deltaX", delta.getX());
    SmartDashboard.putNumber("raw atan (deg)", desiredAngleDeg);
    if (RobotConstants.robotColor != null){
      double tempDangle = 180 + desiredAngleDeg;
      setDesiredAngle(tempDangle >= 180.0 ? tempDangle - 360.0 : tempDangle);
    } else {
      SmartDashboard.putBoolean("Alliance Color Error", true);
    }
  }

  public InstantCommand autoYawInstantCommand(){
    return new InstantCommand(() -> autoYaw(), this);
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

    return m_AutoOrientPID.calculate(getPose().getRotation().getDegrees()); 
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

  public void setRobotSpeeds(ChassisSpeeds speeds){
    currentSpeeds = speeds;
  }

  public ChassisSpeeds getRobotSpeeds(){
    return currentSpeeds;
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    // SwerveModuleState[] moduleStates = SwerveConstants.kinematics.toSwerveModuleStates(
    //                                                   ChassisSpeeds.fromFieldRelativeSpeeds(
    //                                                     speeds, getPoseYaw()));
    SwerveModuleState[] moduleStates = SwerveConstants.kinematics.toSwerveModuleStates(
                                        ChassisSpeeds.discretize(speeds, 0.02));
    setModuleStates(moduleStates);
  }

  public void setFinalPose(Pose2d finalPose){
    m_FinalPose = finalPose;
  }

  public void overrideGyroAngle(double angle){
    m_Gyro.setYaw(angle);
  }


  @Override
  public void periodic() {
    m_SpeakerCenter = RobotConstants.robotColor == Alliance.Blue ? 
                                    FieldConstants.blueSpeakerCenter.toTranslation2d() :
                                    FieldConstants.redSpeakerCenter.toTranslation2d();
    // This method will be called once per scheduler run
    switch (m_state) {
      case ALIGNING_TO_DPAD:
        SmartDashboard.putString("State", "Aligning to DPAD");
        break;
      case SHOOTER_PREP:
        autoYaw();
        SmartDashboard.putString("State", "Shooter Prep");
        break;
      case DRIVER_CONTROL:
        SmartDashboard.putString("State", "Driver Control");
        break;
      default:
        SmartDashboard.putBoolean("STATE ERROR", true);
        break;
    }
    // autoYaw();

    m_PoseEstimator.update(m_Gyro.getRotation2d(), m_ModulePositions);
    
    SmartDashboard.putNumber("Est X Position", getPose().getX());
    SmartDashboard.putNumber("Est Y Position", getPose().getY());
    SmartDashboard.putNumber("Est Pose Yaw from ZeroGyro", getPose().getRotation().getDegrees());
    SmartDashboard.putBoolean("SLOW DRIVE MODE", m_LinearThrottle == Throttle.LINEAR);
    SmartDashboard.putBoolean("SLOW ROT MODE", m_AngularThrottle == Throttle.LINEAR);
    SmartDashboard.putNumber("fieldDist", m_SpeakerCenter.getDistance(getPose().getTranslation()));
    
    updateModulePositions();
    double absAngle = getPose().getRotation().getDegrees();
    absAngle = absAngle >= 0 ? absAngle : absAngle + 360.0;

    m_Field.setRobotPose(getPose().getX(), getPose().getY(), Rotation2d.fromDegrees(absAngle));

    SmartDashboard.putData("Robot Position", m_Field);
    SmartDashboard.putNumber("Distance Error", getPose().getTranslation().getDistance(m_FinalPose.getTranslation()));

    SmartDashboard.putBoolean("GyroInvert", m_ToggleGyro);
  }
}
