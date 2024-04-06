// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.UnitConstants;
import frc.robot.Constants.ShooterConstants.FlywheelSetpoints;
import frc.robot.Constants.ShooterConstants.LeftFlywheelMotorConstants;
import frc.robot.Constants.ShooterConstants.RightFlywheelMotorConstants;
import frc.robot.Constants.ShooterConstants.WristMathConstants;
import frc.robot.Constants.ShooterConstants.WristMotorConstants;
import frc.robot.Constants.ShooterConstants.WristSepoints;
import frc.robot.Constants.SurfaceSpeed;
import frc.robot.HardwareConfig;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class subsystem_Shooter extends SubsystemBase {
  /** Creates a new subsystem_Shooter. */
  private TalonFX m_LeftFlywheel;
  private TalonFX m_RightFlywheel;
  private TalonFX m_Feeder;
  private TalonFX m_Wrist;
  private Timer m_wristTimer = new Timer();
  private Timer m_flywheelTimer = new Timer();
  private InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();


  // private DutyCycleEncoder m_AbsEncoder;
  private ArmFeedforward m_WristFeedforward;
  private HardwareConfig m_Settings;

  private boolean max_speed_wheel = false;

  private double m_DesiredFlywheelSpeed = FlywheelSetpoints.StowSpeed;
  private double m_DesiredShootAngle = WristSepoints.minShootAngle;

  private double m_WristStartPos;
  private double[] rad_list = {1.07, 1.307, 1.57, 
                              1.809}; // TODO: update values here after collecting empirically
                              // 1.8, 2.0, 2.22, 2.75, 4.0
  private double[] angle_list = {51.8, 45.3, 41.5, 
                                38.5};
                                // 40.0, 39.0, 35.0, 30.0, 25.0


//   30 center

// 1.09, subwoofer, 50

// 1.21, 48.01
// 1.51, 47.25
// 1.75, 42.5
// 2.0, 39

// 2.22, 39.5
// 2.296, 38.57
// 2.75, 34.2
// 3.06, 31.5
// 3.48, 29.75
// 3.746, 28.9
// 4.00, 28.4

// Starting Pos - 8.67

  private TrapezoidProfile.State m_previousState = new TrapezoidProfile.State(WristSepoints.minShootAngle, 0);
  private Timer m_TrapezoidTimer = new Timer();
  private Timer m_FlySpikeTimer = new Timer();
  Translation3d speakerCenter3D;
  Translation2d speakerCenter2D;

  public subsystem_Shooter() {
    speakerCenter3D = RobotConstants.robotColor == Alliance.Red ? 
                                    FieldConstants.redSpeakerCenter : 
                                    FieldConstants.blueSpeakerCenter;
    speakerCenter2D = speakerCenter3D.toTranslation2d();


    map.put(rad_list[0], angle_list[0]);
    map.put(rad_list[1], angle_list[1]);
    map.put(rad_list[2], angle_list[2]);
    map.put(rad_list[3], angle_list[3]);

    // for(int i = 0; i < rad_list.length; i++){
    //   map.put(rad_list[i], angle_list[i]);
    // }


    m_LeftFlywheel = new TalonFX(LeftFlywheelMotorConstants.leftFlywheelID, RobotConstants.CANBus);
    m_RightFlywheel = new TalonFX(RightFlywheelMotorConstants.rightFlywheelID, RobotConstants.CANBus);
    m_Feeder = new TalonFX(ShooterConstants.feederMotorID, RobotConstants.CANBus);
    m_Wrist = new TalonFX(WristMotorConstants.wristID, RobotConstants.CANBus);

    // m_AbsEncoder = new DutyCycleEncoder(ShooterConstants.absEncoderChannel);

    m_WristFeedforward = new ArmFeedforward(WristMotorConstants.wristKS, 
    WristMotorConstants.wristKV, 
    WristMotorConstants.wristKG,
    // * Math.cos(MotorRotToShootAngle(m_Wrist.getPosition().getValueAsDouble()) * UnitConstants.DEG_TO_RAD),
    // get max kg mult by costheta
    WristMotorConstants.wristKA);
    
    m_Settings = new HardwareConfig();

    m_LeftFlywheel.getConfigurator().apply(m_Settings.getMotorConfig(3));
    m_RightFlywheel.getConfigurator().apply(m_Settings.getMotorConfig(4));
    m_Wrist.getConfigurator().apply(m_Settings.getMotorConfig(5));

    m_LeftFlywheel.setInverted(true);
    m_RightFlywheel.setInverted(false);

    m_Wrist.setNeutralMode(NeutralModeValue.Brake);
    m_Feeder.setNeutralMode(NeutralModeValue.Coast);

    m_WristStartPos = m_Wrist.getPosition().getValueAsDouble();
    m_TrapezoidTimer.start();
    m_FlySpikeTimer.start();

    m_Feeder.setInverted(true);

    ShooterConstants.fillWristFeedForward();
    m_Wrist.setPosition(0.0);
    SmartDashboard.putNumber("desired shoot angle", m_DesiredShootAngle);
    SmartDashboard.putData("zero shooter wrist", zeroShooterWrist());
    m_flywheelTimer.start();
    m_wristTimer.start();
  }

  public void setFlywheelSpeed(double velocity){
    m_DesiredFlywheelSpeed = velocity;
    SmartDashboard.putNumber("desired wheel speed", velocity);
    // m_DesiredFlywheelSpeed = velocity / (SurfaceSpeed.shooter_transfer * SurfaceSpeed.shooter_rad*SurfaceSpeed.shooter_GR * 2.0 * Math.PI); //gets rpms
  }

  public void stopFlywheels(){
    m_LeftFlywheel.stopMotor();
    m_RightFlywheel.stopMotor();
  }

  public void setFlywheels(){
    m_LeftFlywheel.set(0.9);
    m_RightFlywheel.set(0.9);
  }

  public Command flysStartEnd(){
    return new StartEndCommand(() -> setFlywheels(), () -> stopFlywheels(), this);
  }

  public InstantCommand setFlywheelSpeedCommand(DoubleSupplier velocity){
    return new InstantCommand(() -> setFlywheelSpeed(velocity.getAsDouble()), this);
  }
  
  public boolean flywheelsAtDesiredSpeed(){
    return((Math.abs(m_RightFlywheel.getVelocity().getValueAsDouble() - 
    (m_DesiredFlywheelSpeed - ShooterConstants.flywheelDifferential)) 
    <= FlywheelSetpoints.FlywheelTolerance) && 
    (Math.abs(m_LeftFlywheel.getVelocity().getValueAsDouble() - m_DesiredFlywheelSpeed)
    <= FlywheelSetpoints.FlywheelTolerance));
  }
  
  public double getFlywheelSpeed(){
    return m_DesiredFlywheelSpeed;
  }
  // V = d * kS + d' * kV + d'' * kA
  
  public double getWristPos(){
    return m_Wrist.getPosition().getValueAsDouble();
  }

  public void setWristAngle(){
    m_DesiredShootAngle = 0.0;
  }

  public Command initialSetWristAngle(){
    return Commands.sequence(
      new InstantCommand(() -> setWristAngle(), this),
      Commands.waitSeconds(0.2),
      new InstantCommand(() -> m_Wrist.setPosition(0.0), this)
    );
  }

  public void setDesiredShootAngle(double angle) {
    SmartDashboard.putNumber("inputAngle", angle);
    if (angle <= ShooterConstants.WristSepoints.maxShootAngle && angle >= ShooterConstants.WristSepoints.minShootAngle){
      m_DesiredShootAngle = angle;
      m_TrapezoidTimer.restart();
      SmartDashboard.putNumber("inputAngle2", angle);
    }
  }
  
  public boolean isWristAtDesiredPosition() {
    return Math.abs(shootAngletoMotorRot(m_DesiredShootAngle) - m_Wrist.getPosition().getValueAsDouble()) <= ShooterConstants.wristTolerance;
  }
  
  public void sourceFeeder(){
    m_Feeder.set(-0.4);
  }

  // public InstantCommand manualWrist(DoubleSupplier wristJoystickInput, BooleanSupplier max_speed){
  //   double m_newDesired;
  //   if (Math.abs(wristJoystickInput.getAsDouble())>ControllerConstants.deadband){
  //    m_newDesired = m_DesiredShootAngle + wristJoystickInput.getAsDouble() * ShooterConstants.max_manual_ratio;
  //   if (max_speed.getAsBoolean()){
  //           max_speed_wheel = max_speed.getAsBoolean();
  //   }} else{
  //      m_newDesired = m_Wrist.getPosition().getValueAsDouble();
  //   }
  //   return new InstantCommand(() -> setDesiredShootAngle(m_newDesired), this);
  // }

  
  public void runFeeder(){
    // double feeder_rpm = SurfaceSpeed.feeder_max_surface * 60.0 / (SurfaceSpeed.feeder_rad*SurfaceSpeed.feeder_GR * 2.0 * Math.PI);
    // double feeder_percent_out = feeder_rpm / SurfaceSpeed.feeder_max_rpm;

    m_Feeder.set(SurfaceSpeed.feeder_percent);
    // m_Feeder.setControl(new VelocityVoltage(ShooterConstants.feederRPS));
  }

  // public Command ampScoreShooter(){
  //   return Commands.sequence(
  //     setFlywheelSpeedCommand(null)
  //   )
  // }
  
  public void stopFeeder(){
    m_Feeder.stopMotor();
  }

  public StartEndCommand forwardsFeederStartEnd(){
    return new StartEndCommand(() -> runFeeder(), () -> stopFeeder(), this);
  }

  public StartEndCommand sourceFeederStartEnd(){
    return new StartEndCommand(() -> sourceFeeder(), () -> stopFeeder(), this);
  }

  public Command stowShooter(){
    return Commands.sequence(
      new InstantCommand(() -> stopFeeder()), 
    // new InstantCommand(() -> setDesiredShootAngle(WristSepoints.minShootAngle - 1.5), this),
    new InstantCommand (() -> setFlywheelSpeed(FlywheelSetpoints.StowSpeed), this),
    // Commands.waitSeconds(1.5), 
    new InstantCommand(() -> setDesiredShootAngle(WristSepoints.minShootAngle), this)
    );
    
  }

  public Command stowShooterSource(){
    return Commands.sequence(
    // new InstantCommand(() -> setDesiredShootAngle(WristSepoints.minShootAngle - 1.5), this),
    new InstantCommand (() -> setFlywheelSpeed(FlywheelSetpoints.SourceSpeed), this),
    // Commands.waitSeconds(1.5), 
    new InstantCommand(() -> setDesiredShootAngle(WristSepoints.minShootAngle), this)
    );
    
  }

  public Command stowAndStopShooter(){
    return Commands.sequence(
      new InstantCommand(() -> stopFeeder()), 
    // new InstantCommand(() -> setDesiredShootAngle(WristSepoints.minShootAngle - 1.5), this),
    new InstantCommand (() -> setFlywheelSpeed(FlywheelSetpoints.StowSpeed), this),
    // Commands.waitSeconds(1.5), 
    new InstantCommand(() -> setDesiredShootAngle(WristSepoints.minShootAngle), this)
    );
    
  }
  public Command waitUntilReady(){
    if(RobotConstants.robotColor == null){
      SmartDashboard.putBoolean("Alliance Color Error", true);
    }
    // SmartDashboard.putBoolean("isWithinPos", speakerCenter2D.getDistance(drivePose.getTranslation()) 
    //                                               <= ShooterConstants.shootRadius);
    // Field2d speakerAsField = new Field2d();
    // Pose2d speakerAsPose = new Pose2d(speakerCenter2D, Rotation2d.fromDegrees(0));
    // speakerAsField.setRobotPose(speakerAsPose);
    // SmartDashboard.putData("Speaker Position Shooter", speakerAsField);
    // if (RobotConstants.robotColor == null){
    //   SmartDashboard.putString("alliance", "Eror");
    // } else if (RobotConstants.robotColor == Alliance.Blue){
    //   SmartDashboard.putString("alliance", "Blue");
    // } else if (RobotConstants.robotColor == Alliance.Red){
    //   SmartDashboard.putString("alliance", "Red");
    // } else {
    //   SmartDashboard.putString("alliance", "Mega ERROR");
    // }
    return new WaitUntilCommand(isReadyToShoot());
  }

  public BooleanSupplier isReadyToShoot(){
    return () -> (m_flywheelTimer.get() >= ShooterConstants.shootTime && 
                  m_wristTimer.get() >= ShooterConstants.shootTime);
  }

  public Command autoSourceShooter(){
    return Commands.sequence(
      this.runOnce(() -> setDesiredShootAngle(WristSepoints.sourceWrist)), 
      setFlywheelSpeedCommand(() -> FlywheelSetpoints.SourceSpeed), 
      new InstantCommand(() -> sourceFeeder(), this), 
      new WaitUntilCommand(isReadyToShoot()), 
      new WaitUntilCommand(() -> (m_FlySpikeTimer.get() >= ShooterConstants.flywheelSpikeDebounce)), 
      stowShooterSource());
  }

  public Command sourceShooterStart(){
    return Commands.sequence(
      this.runOnce(() -> setDesiredShootAngle(WristSepoints.sourceWrist)), 
      new InstantCommand(() -> sourceFeeder(), this), 
      setFlywheelSpeedCommand(() -> FlywheelSetpoints.SourceSpeed));
  }

  public Command scoreAmpCommand(){
    return Commands.sequence(
      this.runOnce(() -> setDesiredShootAngle(WristSepoints.ampWrist)),
      setFlywheelSpeedCommand(() -> FlywheelSetpoints.AmpSpeed),
      waitUntilReady(), 
      new InstantCommand(() -> runFeeder(), this),
      Commands.waitSeconds(0.5),
      stowShooter()
    ); 
  }

  // public Commands shooterAmpFling(){
  //   return Commands.sequence(null)
  // }

  public double MotorRotToShootAngle(double motorRot){
    //https://replit.com/@matthewk126/rizzler#src/main/java/Main.java
    // motorRot-=ShooterConstants.driverArmOffset;
    double r1 = WristMathConstants.linkageLength;
    double r2 = WristMathConstants.driveArmLength;
    double theta =((180.0 + WristMathConstants.driverArmOffset) - (motorRot / WristMathConstants.wristGearRatio) * 360.0) * UnitConstants.DEG_TO_RAD;
    double theta_deg = theta * UnitConstants.RAD_TO_DEG;
    // (((180+50.0177)-(motorRot/20.57)*360))*(Math.PI/180.0);
    double l1 = WristMathConstants.shooterConnectionLength;
    double[] P4 = {-r2 * Math.cos(theta), -r2 * Math.sin(theta)};
    double[] P1 = {6.75, 0};
    double d_14 = Math.hypot(P1[0] - P4[0], P1[1] - P4[1]);
    double beta = Math.acos((Math.pow(d_14,2) + Math.pow(l1,2) - Math.pow(r1,2))
                            / (2.0 * d_14 * l1));
    double d_45 = Math.hypot(P4[0], P4[1]);
    double gamma = Math.acos((Math.pow(d_14,2) + Math.pow(P1[0],2) - Math.pow(d_45,2)) 
                            / (2.0 * d_14 * P1[0]));
    double shotAngle = ((beta - gamma) * UnitConstants.RAD_TO_DEG) - WristMathConstants.shooterAngleToPivotAngle;
    if (theta_deg < 180.0){
      shotAngle = ((beta + gamma) * UnitConstants.RAD_TO_DEG) - WristMathConstants.shooterAngleToPivotAngle;
    }
    return shotAngle;
  }

  public double shootAngletoMotorRot(double shootAngle){
    //https://www.desmos.com/calculator/znscicoddu
    double netAngle = (shootAngle + WristMathConstants.shooterAngleToPivotAngle) * UnitConstants.DEG_TO_RAD;
    SmartDashboard.putNumber("netAngle", netAngle);
    double x1 = WristMathConstants.shooterConnectionLength * Math.cos(netAngle) - 6.75;
    double y1 = WristMathConstants.shooterConnectionLength * Math.sin(netAngle);
    SmartDashboard.putNumberArray("x1, y1", new double[] {x1, y1});
    double r1 = WristMathConstants.linkageLength;
    double r2 = WristMathConstants.driveArmLength;
    double D = Math.hypot(x1, y1);
    double sumSqrs = (r1 * r1 + r2 * r2) / (D * D);
    double difSqrsR2R1 = (r2 * r2 - r1 * r1) / (D * D);
    double k1 = difSqrsR2R1 / 2.0 + 0.5;
    double k2 = 0.5 * Math.sqrt(2.0 * sumSqrs - Math.pow(difSqrsR2R1, 2) - 1);
    double[] BasePoint = {x1 * k1 - y1 * k2, y1 * k1 + x1 * k2};
    /* double[] BasePoint = {
      x1 * (((r2 * r2 - r1 * r1) / (2 * D * D)) + 0.5) 
      - y1 * 0.5 * Math.sqrt(2 * ((r1 * r1 + r2 * r2)/(D * D)) - ((Math.pow((r1 * r1 - r2 * r2), 2)) / (D * D * D * D)) - 1), 
      y1 * (((r2 * r2 - r1 * r1) / (2 * D * D)) + 0.5) 
      + x1 * 0.5 * Math.sqrt(2 * ((r1 * r1 + r2 * r2) / (D * D)) - ((Math.pow((r1 * r1 - r2 * r2), 2)) / (D * D * D * D)) - 1), 
    }; */
    double atan2Deg =  Math.atan2(BasePoint[1], BasePoint[0]) * UnitConstants.RAD_TO_DEG;
    SmartDashboard.putNumber("atan2deg", atan2Deg);

    double atan2Normalized = atan2Deg < 0 ? atan2Deg + 180 : atan2Deg - 180;
    double linkageAngleFromStart = WristMathConstants.driverArmOffset - atan2Normalized;
    double motorPosRotations = linkageAngleFromStart * WristMathConstants.wristGearRatio * UnitConstants.DEG_TO_ROT;
    return motorPosRotations;
  }


// public void add_data(Pose2d drivePose, double angle){
  // }

  public double get_from_table(double drivePose){
    return map.get(drivePose);
  }

  @Override
  public void periodic() {
    if (!isWristAtDesiredPosition()){
      m_wristTimer.restart();
    }
    if (!flywheelsAtDesiredSpeed()){
      m_flywheelTimer.restart();
    }
    //TODO: TODO: TODO: Comment back out b4 match
    // m_DesiredShootAngle = SmartDashboard.getNumber("desired shoot angle", m_DesiredShootAngle);
    SmartDashboard.putNumber("desired shoot angle", m_DesiredShootAngle);
    if (m_LeftFlywheel.getStatorCurrent().getValueAsDouble() <= ShooterConstants.flywheelSourceCurrent
    &&  m_RightFlywheel.getStatorCurrent().getValueAsDouble() <= ShooterConstants.flywheelSourceCurrent){
      m_FlySpikeTimer.restart();
    }
    SmartDashboard.putBoolean("flySpikeLeft", m_LeftFlywheel.getStatorCurrent().getValueAsDouble() > ShooterConstants.flywheelSourceCurrent);
    SmartDashboard.putBoolean("flySpikeRight", m_RightFlywheel.getStatorCurrent().getValueAsDouble() > ShooterConstants.flywheelSourceCurrent);
    SmartDashboard.putBoolean("flySpike", m_FlySpikeTimer.get() >= ShooterConstants.flywheelSpikeDebounce);
    SmartDashboard.putNumber("flySpikeTimer", m_FlySpikeTimer.get());
    SmartDashboard.putBoolean("readyToShoot", isReadyToShoot().getAsBoolean());
    SmartDashboard.putNumber("left flywheel current", m_LeftFlywheel.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("right flywheel current", m_RightFlywheel.getStatorCurrent().getValueAsDouble());

    speakerCenter3D = DriverStation.getAlliance().isPresent() ? 
                                    DriverStation.getAlliance().get() == Alliance.Red ? 
                                    FieldConstants.redSpeakerCenter : 
                                    FieldConstants.blueSpeakerCenter :
                      new Translation3d();
    speakerCenter2D = speakerCenter3D.toTranslation2d();
    speakerCenter2D = speakerCenter3D.toTranslation2d();
    SmartDashboard.putBoolean("shooter wrist ok", isWristAtDesiredPosition());
    SmartDashboard.putNumber("shooter wrist encoder", m_Wrist.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("desired Shooter motor Angle", shootAngletoMotorRot(m_DesiredShootAngle));
    SmartDashboard.putNumber("Shooter Wrist Position", MotorRotToShootAngle(m_Wrist.getPosition().getValueAsDouble()));
    SmartDashboard.putNumber("feeder speed", m_Feeder.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("feeder get", m_Feeder.get());

    double wFF = m_WristFeedforward.calculate(m_DesiredShootAngle*UnitConstants.DEG_TO_RAD, 
    Units.degreesToRadians(MotorRotToShootAngle(m_Wrist.getVelocity().getValueAsDouble())));
    // double wFF = ShooterConstants.wristFeedForward.get(shootAngletoMotorRot(m_DesiredShootAngle));
    SmartDashboard.putNumber("wristFeedForward", wFF);
    SmartDashboard.putNumber("Shooter PrevStatePos", m_previousState.position);
    SmartDashboard.putNumber("Shooter PrevStateVel", m_previousState.velocity);
    if(m_DesiredShootAngle == 0.0){
      m_Wrist.set(-0.2);
    } else {
      if (m_DesiredShootAngle != ShooterConstants.WristSepoints.minShootAngle) {
        m_Wrist.setControl(new MotionMagicVoltage((shootAngletoMotorRot(m_DesiredShootAngle))).withSlot(0)
        .withFeedForward(wFF));
      } else {
        m_Wrist.stopMotor();
      }
    }
      
    SmartDashboard.putBoolean("wheels ok", flywheelsAtDesiredSpeed());
    SmartDashboard.putNumber("wheel desired", m_DesiredFlywheelSpeed);
    SmartDashboard.putNumber("flywheel1", m_LeftFlywheel.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("flywheel2", m_RightFlywheel.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Wrist Start", m_WristStartPos);

    

    if (max_speed_wheel){
      m_LeftFlywheel.set(1);
    } else {
      m_LeftFlywheel.setControl(new VelocityVoltage(m_DesiredFlywheelSpeed));
    }

    if (max_speed_wheel){
      m_RightFlywheel.set(0.95);
    } else {
      if(m_DesiredFlywheelSpeed >= ShooterConstants.flywheelDifferential){
        m_RightFlywheel.setControl(new VelocityVoltage(m_DesiredFlywheelSpeed - ShooterConstants.flywheelDifferential));
      } else if (m_DesiredFlywheelSpeed == FlywheelSetpoints.SourceSpeed){
        m_RightFlywheel.setControl(new VelocityVoltage(m_DesiredFlywheelSpeed));
      }
    }
  }

  public void zeroWrist(){
    m_Wrist.setPosition(0.0);
  }

  public Command zeroShooterWrist(){
    return new InstantCommand(() -> zeroWrist(), this).ignoringDisable(true);
  }
}
