// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import java.util.Arrays;

import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.controls.VoltageOut;

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
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
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
import edu.wpi.first.units.Voltage;


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
  
  private boolean m_IsAutoOrient;
  private boolean m_IsPark;
  
  private int m_DPAD;
  private int m_OrientCounter;

  private Vector<N3> vec1 = VecBuilder.fill(0.7, 0.7, 0.1);
  private Vector<N3> vec2 = VecBuilder.fill(0.3, 0.3, 0.9);

  private Field2d m_Field;
  private SysIdRoutine m_driveSysID;
  private SysIdRoutine m_angleSysID;

  private ChassisSpeeds currentSpeeds;
  
  private final VoltageOut m_sysidControl = new VoltageOut(0);

  public subsystem_DriveTrain() {
    SignalLogger.enableAutoLogging(true);
    SignalLogger.start();
    m_Gyro = new Pigeon2(SwerveConstants.gyroID, SwerveConstants.CANBus);
    m_FrontLeft = new SwerveModule(FrontLeftModule.constants);
    m_FrontRight = new SwerveModule(FrontRightModule.constants);
    m_BackLeft = new SwerveModule(BackLeftModule.constants);
    m_BackRight = new SwerveModule(BackRightModule.constants);
    m_Kinematics = new SwerveDriveKinematics(SwerveConstants.frontLeft, 
                                            SwerveConstants.frontRight,
                                            SwerveConstants.backLeft,
                                            SwerveConstants.backRight);

    m_driveSysID = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,         // Default ramp rate is acceptable
          Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
          null,          // Default timeout is acceptable
          (state)->SignalLogger.writeString("state", state.toString())), // Log state with Phoenix SignalLogger class
      new SysIdRoutine.Mechanism(
          (Measure<Voltage> volts)-> setDriveMotorVoltages(m_sysidControl.withOutput(volts.in(Volts))),
          null, 
          this));

    m_angleSysID = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,         // Default ramp rate is acceptable
          Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
          null,          // Default timeout is acceptable
          (state)->SignalLogger.writeString("state", state.toString())), // Log state with Phoenix SignalLogger class
      new SysIdRoutine.Mechanism(
          (Measure<Voltage> volts)-> setAngleMotorVoltages(m_sysidControl.withOutput(volts.in(Volts))),
          null,
          this));

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

    currentSpeeds = new ChassisSpeeds();

    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetOdometry,
      this::getRobotSpeeds,
      this::driveRobotRelative,
      new HolonomicPathFollowerConfig(AutoConstants.MaxSpeedMetersPerSecond * SwerveConstants.driveGearRatio,
      Math.hypot(10.375, 10.375),
      new ReplanningConfig()),
    () -> {
      var alliance = DriverStation.getAlliance();
      return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    },
    this);

  }

  public void updateModulePositions(){
    m_ModulePositions[0] = m_FrontLeft.getPosition();
    m_ModulePositions[1] = m_FrontRight.getPosition();
    m_ModulePositions[2] = m_BackLeft.getPosition();
    m_ModulePositions[3] = m_BackRight.getPosition();
  }

  public void setDriveMotorVoltages(VoltageOut v){
    m_FrontLeft.setDriveVoltage(v);
    m_FrontRight.setDriveVoltage(v);
    m_BackLeft.setDriveVoltage(v);
    m_BackRight.setDriveVoltage(v);
  }

  public void setAngleMotorVoltages(VoltageOut v){
    m_FrontLeft.setTurnVoltage(v);
    m_FrontRight.setTurnVoltage(v);
    m_BackLeft.setTurnVoltage(v);
    m_BackRight.setTurnVoltage(v);
  }

  public Command driveQuasistatic(SysIdRoutine.Direction direction) {
    return m_driveSysID.quasistatic(direction);
  }
  public Command driveDynamic(SysIdRoutine.Direction direction) {
      return m_driveSysID.dynamic(direction);
  }

  public Command angleQuasistatic(SysIdRoutine.Direction direction) {
      return m_angleSysID.quasistatic(direction);
  }
  public Command angleDynamic(SysIdRoutine.Direction direction) {
      return m_angleSysID.dynamic(direction);
  }

  public Command zeroAngleMotors(){
    return this.runOnce(() -> setAngleMotorVoltages(new VoltageOut(0))).repeatedly();
  }

  public void swerveDrive(double xSpeedMetersPerSecond, 
                          double ySpeedMetersPerSecond, 
                          double zRotRadiansPerSecond, 
                          boolean isFieldRelative, 
                          boolean isOpenLoop,
                          boolean isAuto){
    
    zRotRadiansPerSecond = m_IsAutoOrient ? getAngularVelocity() : zRotRadiansPerSecond;
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
    
    double maxVelocity = isAuto ? AutoConstants.MaxSpeedMetersPerSecond : SwerveConstants.maxSpeed * 4.0;
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

  public InstantCommand autoTestCmd(String key, double val){
    return new InstantCommand(() -> SmartDashboard.putNumber(key, val), this);
  }

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

  

  public void togglePark(){
    m_IsPark = !m_IsPark;
  }

  public InstantCommand toggleParkCommand(){
    return new InstantCommand(() -> togglePark(), this);
  }

  public void setTrajectory(ChoreoTrajectory trajectory){
    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.MaxSpeedMetersPerSecond, 
                                                  AutoConstants.MaxAccelMetersPerSecondSquared);
    Trajectory traj = TrajectoryGenerator.generateTrajectory(Arrays.asList(trajectory.getPoses()), config);
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
    SmartDashboard.putNumber("Est X Position", m_PoseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Est Y Position", m_PoseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Est Pose Yaw", m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees());
    SmartDashboard.putBoolean("SLOW DRIVE MODE", m_LinearThrottle == Throttle.LINEAR);
    SmartDashboard.putBoolean("SLOW ROT MODE", m_AngularThrottle == Throttle.LINEAR);
    // print(0);
    // print(1);
    // print(2);
    // print(3);
    m_Field.setRobotPose(m_PoseEstimator.getEstimatedPosition());
    m_PoseEstimator.update(m_Gyro.getRotation2d(), m_ModulePositions);
    updateModulePositions();
    SmartDashboard.putData("field", m_Field);
  }
}
