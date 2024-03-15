// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ControllerConstants;
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
import frc.robot.HardwareConfig;

public class subsystem_Shooter extends SubsystemBase {
  /** Creates a new subsystem_Shooter. */
  private TalonFX m_LeftFlywheel;
  private TalonFX m_RightFlywheel;
  private TalonFX m_Feeder;
  private TalonFX m_Wrist;
  
  // private DutyCycleEncoder m_AbsEncoder;
  private TrapezoidProfile m_wristProfile;
  private ArmFeedforward m_WristFeedforward;
  private HardwareConfig m_Settings;

  private boolean max_speed_wheel = false;

  private double m_DesiredFlywheelSpeed = 0.0;
  private double m_DesiredShootAngle = WristSepoints.minShootAngle;

  private double m_WristStartPos;

  private TrapezoidProfile.State m_previousState = new TrapezoidProfile.State(WristSepoints.minShootAngle, 0);
  private Timer m_TrapezoidTimer = new Timer();

  Translation3d speakerCenter3D;
  Translation2d speakerCenter2D;

  public subsystem_Shooter() {
    speakerCenter3D = RobotConstants.robotColor == Alliance.Red ? 
                                    FieldConstants.redSpeakerCenter : 
                                    FieldConstants.blueSpeakerCenter;
    speakerCenter2D = speakerCenter3D.toTranslation2d();

    m_LeftFlywheel = new TalonFX(LeftFlywheelMotorConstants.leftFlywheelID, RobotConstants.CANBus);
    m_RightFlywheel = new TalonFX(RightFlywheelMotorConstants.rightFlywheelID, RobotConstants.CANBus);
    m_Feeder = new TalonFX(ShooterConstants.feederMotorID, RobotConstants.CANBus);
    m_Wrist = new TalonFX(WristMotorConstants.wristID, RobotConstants.CANBus);

    // m_AbsEncoder = new DutyCycleEncoder(ShooterConstants.absEncoderChannel);

    m_WristFeedforward = new ArmFeedforward(WristMotorConstants.wristKS, 
                                            WristMotorConstants.wristKV, 
                                            WristMotorConstants.wristKG,
                                            WristMotorConstants.wristKA);
    m_wristProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(WristMotorConstants.maxWristVelocity,
                                      WristMotorConstants.maxWristAccel));
    
    m_Settings = new HardwareConfig();

    m_LeftFlywheel.getConfigurator().apply(m_Settings.getMotorConfig(3));
    m_RightFlywheel.getConfigurator().apply(m_Settings.getMotorConfig(4));
    m_Wrist.getConfigurator().apply(m_Settings.getMotorConfig(5));

    m_LeftFlywheel.setInverted(true);
    m_RightFlywheel.setInverted(false);

    m_WristStartPos = m_Wrist.getPosition().getValueAsDouble();
    m_TrapezoidTimer.start();

    m_Feeder.setInverted(true);
  }

  public void setFlywheelSpeed(double velocity){
    m_DesiredFlywheelSpeed = velocity;
  }

  public void stopFlywheels(){
    m_LeftFlywheel.stopMotor();
    m_RightFlywheel.stopMotor();
  }

  public void setFlywheelsTest(){
    m_LeftFlywheel.set(0.9);
    m_RightFlywheel.set(0.9);
  }

  public Command flysStartEnd(){
    return new StartEndCommand(() -> setFlywheelsTest(), () -> stopFlywheels(), this);
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
  
  public void setDesiredShootAngle(double angle) {
    SmartDashboard.putNumber("inputAngle", angle);
    if (angle <= ShooterConstants.WristSepoints.maxShootAngle && angle >= ShooterConstants.WristSepoints.minShootAngle){
      m_DesiredShootAngle = angle;
      m_TrapezoidTimer.restart();
    }
  }
  
  public boolean isWristAtDesiredPosition() {
    return Math.abs(m_Wrist.getPosition().getValueAsDouble() - shootAngletoMotorRot(m_DesiredShootAngle)) <= ShooterConstants.wristTolerance;
  }
  
  public void sourceFeeder(){
    m_Feeder.set(-0.5);
  }
  
  public void runFeeder(){
    m_Feeder.set(1);//ahhhh
  }
  
  public void stopFeeder(){
    m_Feeder.stopMotor();
  }

  public StartEndCommand forwardsFeederStartEnd(){
    return new StartEndCommand(() -> runFeeder(), () -> stopFeeder(), this);
  }

  public StartEndCommand sourceFeederStartEnd(){
    return new StartEndCommand(() -> sourceFeeder(), () -> stopFeeder(), this);
  }
  
  public Command waitUntilReady(Pose2d drivePose){
    if(RobotConstants.robotColor == null){
      SmartDashboard.putBoolean("Alliance Color Error", true);
    }
    SmartDashboard.putBoolean("isWithinPos", speakerCenter2D.getDistance(drivePose.getTranslation()) 
                                                  <= ShooterConstants.shootRadius);
    Field2d speakerAsField = new Field2d();
    Pose2d speakerAsPose = new Pose2d(speakerCenter2D, Rotation2d.fromDegrees(0));
    speakerAsField.setRobotPose(speakerAsPose);
    SmartDashboard.putData("Speaker Position Shooter", speakerAsField);
    if (RobotConstants.robotColor == null){
      SmartDashboard.putString("alliance", "Eror");
    } else if (RobotConstants.robotColor == Alliance.Blue){
      SmartDashboard.putString("alliance", "Blue");
    } else if (RobotConstants.robotColor == Alliance.Red){
      SmartDashboard.putString("alliance", "Red");
    } else {
      SmartDashboard.putString("alliance", "Mega ERROR");
    }

    boolean condition = (speakerCenter2D.getDistance(drivePose.getTranslation()) 
                        <= ShooterConstants.shootRadius) 
                        && isWristAtDesiredPosition() && flywheelsAtDesiredSpeed();
    SmartDashboard.putBoolean("readyToShoot", condition);

    return new WaitUntilCommand(() -> condition);
  }

  public InstantCommand manualWrist(DoubleSupplier wristJoystickInput, BooleanSupplier max_speed){
    double m_newDesired;
    if (Math.abs(wristJoystickInput.getAsDouble())>ControllerConstants.deadband){
     m_newDesired = m_DesiredShootAngle + wristJoystickInput.getAsDouble() * ShooterConstants.manualWristRatio;
    if (max_speed.getAsBoolean()){
            max_speed_wheel = max_speed.getAsBoolean();
    }} else{
       m_newDesired = m_Wrist.getPosition().getValueAsDouble();
    }
    return new InstantCommand(() -> setDesiredShootAngle(m_newDesired), this);
  }


  @Override
  public void periodic() {
    speakerCenter3D = RobotConstants.robotColor == Alliance.Red ? 
                                    FieldConstants.redSpeakerCenter : 
                                    FieldConstants.blueSpeakerCenter;
    speakerCenter2D = speakerCenter3D.toTranslation2d();
    speakerCenter2D = speakerCenter3D.toTranslation2d();
    SmartDashboard.putBoolean("shooter wrist ok", isWristAtDesiredPosition());
    SmartDashboard.putNumber("shooter wrist encoder", m_Wrist.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("desired shoot angle", m_DesiredShootAngle);
    SmartDashboard.putNumber("desired Shooter Motor Angle", shootAngletoMotorRot(m_DesiredShootAngle));
    SmartDashboard.putNumber("abs shooter wrist Angle (exp)", MotorRotToShootAngle(m_Wrist.getPosition().getValueAsDouble()));
    SmartDashboard.putNumber("feeder speed", m_Feeder.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("feeder get", m_Feeder.get());
    m_previousState = m_wristProfile.calculate(m_TrapezoidTimer.get(), m_previousState, 
                                    new TrapezoidProfile.State(m_DesiredShootAngle, 0));
    double wFF = m_WristFeedforward.calculate(m_DesiredShootAngle, 
      MotorRotToShootAngle(m_Wrist.getVelocity().getValueAsDouble()));
    SmartDashboard.putNumber("wristFeedForward", wFF / UnitConstants.kNominal);
    SmartDashboard.putNumber("Shooter PrevStatePos", m_previousState.position);
    SmartDashboard.putNumber("Shooter PrevStateVel", m_previousState.velocity);
    if (m_DesiredShootAngle != ShooterConstants.WristSepoints.minShootAngle) {
      m_Wrist.setControl(new PositionVoltage(shootAngletoMotorRot(m_DesiredShootAngle)).withSlot(0)
      .withFeedForward(wFF / UnitConstants.kNominal));
    } 
      else{
      m_Wrist.stopMotor();

      }
    

    SmartDashboard.putBoolean("wheels ok", flywheelsAtDesiredSpeed());
    SmartDashboard.putNumber("wheel desired", m_DesiredFlywheelSpeed);
    SmartDashboard.putNumber("flywheel1", m_LeftFlywheel.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("flywheel2", m_RightFlywheel.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Wrist Start", m_WristStartPos);

    if (max_speed_wheel){
      m_LeftFlywheel.set(1);
    }else if (m_DesiredFlywheelSpeed > m_LeftFlywheel.getVelocity().getValueAsDouble()){
      m_LeftFlywheel.setControl(new VelocityVoltage(m_DesiredFlywheelSpeed));
    }else{
      m_LeftFlywheel.stopMotor();
    }

     if (max_speed_wheel){
      m_RightFlywheel.set(0.95);
    }else if (m_DesiredFlywheelSpeed > m_RightFlywheel.getVelocity().getValueAsDouble()){
    if (m_DesiredFlywheelSpeed >= ShooterConstants.flywheelDifferential){
      m_RightFlywheel.setControl(new VelocityVoltage(m_DesiredFlywheelSpeed - ShooterConstants.flywheelDifferential));
    }
  }
  else{
          m_RightFlywheel.stopMotor();
  }
}

  // Use
  public double shootAngletoMotorRot(double shootAngle){
    //https://www.desmos.com/calculator/znscicoddu
    double netAngle = (shootAngle + WristMathConstants.shooterAngleToPivotAngle) * UnitConstants.DEG_TO_RAD;
    double x1 = WristMathConstants.shooterConnectionLength * Math.cos(netAngle) - 6.75;
    double y1 = WristMathConstants.shooterConnectionLength * Math.sin(netAngle);
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
    double atan2Normalized = atan2Deg < 0 ? atan2Deg + 180 : atan2Deg - 180;
    double linkageAngleFromStart = WristMathConstants.driverArmOffset - atan2Normalized;
    double motorPosRotations = linkageAngleFromStart * WristMathConstants.wristGearRatio * UnitConstants.DEG_TO_ROT;
    return motorPosRotations;
  }

  // Use
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
    double[] P1 = {6.75, 0}; //its 6.75 right? //yes -matthew
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

  // // Don't Use
  // public double MotorRotToShootAngleMatthew(double motorRot){
  //   //https://www.desmos.com/calculator/xdtyyrvaey
  //   double motorDriveAngle = motorRot * 360 / WristMathConstants.wristGearRatio - WristMathConstants.driverArmOffset;
  //   double[] basePoint = {
  //     6.75 - WristMathConstants.driveArmLength * Math.cos(motorDriveAngle * UnitConstants.DEG_TO_RAD),
  //     WristMathConstants.driveArmLength * Math.sin(motorDriveAngle * UnitConstants.DEG_TO_RAD)};
  //   double x1 = basePoint[0];
  //   double y1 = basePoint[1];
  //   double r1 = WristMathConstants.linkageLength;
  //   double r2 = WristMathConstants.shooterConnectionLength;
  //   double D = Math.hypot(x1, y1);
  //   double[] topPoint = {
  //     x1 * (((r2*r2 - r1*r1) / (2*D*D)) + 0.5) 
  //     -y1*0.5*Math.sqrt(2*((r1*r1+r2*r2)/(D*D)) - ((Math.pow((r1*r1-r2*r2), 2))/(D*D*D*D)) - 1), 
  //     y1 * (((r2*r2 - r1*r1) / (2*D*D)) + 0.5) 
  //     +x1*0.5*Math.sqrt(2*((r1*r1+r2*r2)/(D*D)) - ((Math.pow((r1*r1-r2*r2), 2))/(D*D*D*D)) - 1), 
  //   };
  //   double atan2Deg =  Math.atan2(topPoint[1], topPoint[0]) * UnitConstants.RAD_TO_DEG;
  //   return atan2Deg - WristMathConstants.shooterAngleToPivotAngle;
  // }
}
