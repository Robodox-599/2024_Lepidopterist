// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.UnitConstants;
import frc.robot.HardwareConfig;

public class subsystem_Shooter extends SubsystemBase {
  /** Creates a new subsystem_Shooter. */
  private TalonFX m_LeftFlywheel;
  private TalonFX m_RightFlywheel;
  private TalonFX m_Feeder;
  private TalonFX m_Wrist;
  
  // private DutyCycleEncoder m_AbsEncoder;
  private TrapezoidProfile m_Constraints;
  private ArmFeedforward m_WristFeedforward;
  private HardwareConfig m_Settings;

  private boolean m_IsSourcing = false;
  private boolean m_IsFeederGo = false;
  private boolean m_manualFeeder = false;

  private double m_DesiredFlywheelSpeed = 0.0;
  private double m_DesiredShootAngle = 0.0;

  private double m_WristStartPos;


  public subsystem_Shooter() {
    m_LeftFlywheel = new TalonFX(ShooterConstants.leftFlywheelID, RobotConstants.CANBus);
    m_RightFlywheel = new TalonFX(ShooterConstants.rightFlywheelID, RobotConstants.CANBus);
    m_Feeder = new TalonFX(ShooterConstants.feederID, RobotConstants.CANBus);
    m_Wrist = new TalonFX(ShooterConstants.wristID, RobotConstants.CANBus);

    // m_AbsEncoder = new DutyCycleEncoder(ShooterConstants.absEncoderChannel);

    m_WristFeedforward = new ArmFeedforward(ShooterConstants.wristKS, 
                                            ShooterConstants.wristKV, 
                                            ShooterConstants.wristKG,
                                            ShooterConstants.wristKA);
    m_Constraints = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(ShooterConstants.maxWristVelocity,
                                      ShooterConstants.maxWristAccel));
    
    m_Settings = new HardwareConfig();

    m_LeftFlywheel.getConfigurator().apply(m_Settings.getMotorConfig(2));
    m_RightFlywheel.getConfigurator().apply(m_Settings.getMotorConfig(3));
    m_Wrist.getConfigurator().apply(m_Settings.getMotorConfig(4));

    m_WristStartPos = m_Wrist.getPosition().getValueAsDouble();
  }

  public void setFlywheelSpeed(double velocity){
    m_DesiredFlywheelSpeed = velocity;
  }

  public InstantCommand setFlywheelSpeedCommand(DoubleSupplier velocity){
    return new InstantCommand(() -> setFlywheelSpeed(velocity.getAsDouble()), this);
  }
  
  public void sourceFeeder(boolean on){
    m_IsSourcing = on;
  }

  public void toggleFeeder(boolean shoot){
    m_manualFeeder = shoot;
  }

  public InstantCommand toggleFeederCommand(BooleanSupplier shoot){
      return new InstantCommand(() -> toggleFeeder(shoot.getAsBoolean()), this);
  }

  public boolean flywheelAtDesiredSpeed(){
      return((Math.abs(m_RightFlywheel.getVelocity().getValueAsDouble() - 
      (m_DesiredFlywheelSpeed - ShooterConstants.differential)) 
      <= ShooterConstants.FlywheelMargin) && 
      (Math.abs(m_LeftFlywheel.getVelocity().getValueAsDouble() - m_DesiredFlywheelSpeed)
      <= ShooterConstants.FlywheelMargin));
  }

  public double getFlywheelSpeed(){
      return m_DesiredFlywheelSpeed;
  }
  // V = d * kS + d^. * kV + d^.. * kA

  public void setDesiredShootAngle(double angle) {
    SmartDashboard.putNumber("inputAngle", angle);
    if (angle <= ShooterConstants.maxNetAngle && angle >= ShooterConstants.minNetAngle)
      m_DesiredShootAngle = angle;
  }

  public boolean isWristAtDesiredPosition() {
    return Math.abs(m_Wrist.getPosition().getValueAsDouble() - shootAngletoMotorRot(m_DesiredShootAngle)) <= ShooterConstants.wristError;
  }

  public Command waitUntilReady(Pose2d drivePose){
    if(RobotConstants.robotColor == null){
      SmartDashboard.putBoolean("Alliance Color Error", true);
    }
    Translation3d speakerCenter3D = RobotConstants.robotColor == Alliance.Blue ? 
                                    FieldConstants.blueSpeakerCenter : 
                                    FieldConstants.redSpeakerCenter;
    Translation2d speakerCenter2D = speakerCenter3D.toTranslation2d();
    boolean condition = !(speakerCenter2D.getDistance(drivePose.getTranslation()) > ShooterConstants.shootRadius ||
                          !isWristAtDesiredPosition());
    return Commands.sequence(new InstantCommand(() -> {}, this)).until(() -> {return condition;});
  }

  public InstantCommand manualWrist(DoubleSupplier wristOffsetneg1to1){
    if (Math.abs(wristOffsetneg1to1.getAsDouble()) > ShooterConstants.manualWristThreshold){
      double m_newDesired = m_DesiredShootAngle + wristOffsetneg1to1.getAsDouble()*ShooterConstants.max_manual_ratio;
      if (m_newDesired <= ShooterConstants.maxNetAngle && m_newDesired >= ShooterConstants.minNetAngle){
        return new InstantCommand(() -> setDesiredShootAngle(m_newDesired), this);
      }
    }
    return new InstantCommand();
  }

  public StartEndCommand feedersStartEnd(){
    return new StartEndCommand(()->toggleFeeder(true), ()->toggleFeeder(false), this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("wrist ok", isWristAtDesiredPosition());
    SmartDashboard.putNumber("wrist encoder", m_Wrist.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("desired shoot angle", m_DesiredShootAngle);
    SmartDashboard.putNumber("desired Motor Angle", shootAngletoMotorRot(m_DesiredShootAngle));

    if (m_DesiredShootAngle != ShooterConstants.minNetAngle) {
      m_Wrist.setControl(new PositionVoltage(shootAngletoMotorRot(m_DesiredShootAngle)).withSlot(0)
      .withFeedForward(m_WristFeedforward.calculate(m_DesiredShootAngle, 
      MotorRotToShootAngle(m_Wrist.getVelocity().getValueAsDouble()))));
    } else {
      m_Wrist.stopMotor();
    }

    SmartDashboard.putBoolean("wheels ok", flywheelAtDesiredSpeed());
    SmartDashboard.putNumber("wheel desired", m_DesiredFlywheelSpeed);
    SmartDashboard.putNumber("flywheel1", m_LeftFlywheel.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("flywheel2", m_RightFlywheel.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("wrist Start", m_WristStartPos);
    
    // Testing Purposes Only
    m_Feeder.set(m_IsFeederGo ? -0.5 : m_IsSourcing ? 0.5 : 0.0);
    if ((isWristAtDesiredPosition() && flywheelAtDesiredSpeed() && 
    (!(m_DesiredShootAngle == ShooterConstants.minNetAngle+ m_WristStartPos))) || m_manualFeeder){
      m_IsFeederGo = true;
    }
    // Actual Code
    m_LeftFlywheel.setControl(new VelocityVoltage(m_DesiredFlywheelSpeed));
    
    if (m_DesiredFlywheelSpeed - ShooterConstants.differential >= 0){
      m_RightFlywheel.setControl(new VelocityVoltage(m_DesiredFlywheelSpeed - ShooterConstants.differential));
    }
  }

  public double shootAngletoMotorRot(double shootAngle){
    //https://www.desmos.com/calculator/znscicoddu
    double netAngle = (shootAngle+ShooterConstants.shooterAngleToPivotAngle) * UnitConstants.DEG_TO_RAD;
    double[] ConnectionPoint = {ShooterConstants.shooterConnectionLength*Math.cos(netAngle)-6.75, 
    ShooterConstants.shooterConnectionLength*Math.sin(netAngle)};
    double x1 = ConnectionPoint[0];
    double y1 = ConnectionPoint[1];
    double r1 = ShooterConstants.linkageLength;
    double r2 = ShooterConstants.driveArmLength;
    double D = Math.sqrt(x1*x1+y1*y1);
    double[] BasePoint = {
      x1 * (((r2*r2 - r1*r1) / (2*D*D)) + 0.5) 
      -y1*0.5*Math.sqrt(2*((r1*r1+r2*r2)/(D*D)) - ((Math.pow((r1*r1-r2*r2), 2))/(D*D*D*D)) - 1), 
      y1 * (((r2*r2 - r1*r1) / (2*D*D)) + 0.5) 
      +x1*0.5*Math.sqrt(2*((r1*r1+r2*r2)/(D*D)) - ((Math.pow((r1*r1-r2*r2), 2))/(D*D*D*D)) - 1), 
    };
    double atan2Deg =  Math.atan2(BasePoint[1], BasePoint[0]) * UnitConstants.RAD_TO_DEG;
    double atan2Normalized = (atan2Deg < 0)? atan2Deg+180 : atan2Deg-180;
    double linkageAngleFromStart = ShooterConstants.driverArmOffset - atan2Normalized;
    double motorPosRotations = linkageAngleFromStart *(ShooterConstants.wristGearRatio/360);
    return motorPosRotations;
  }
  public double MotorRotToShootAngle(double motorRot){
    //https://replit.com/@matthewk126/rizzler#src/main/java/Main.java
    // motorRot-=ShooterConstants.driverArmOffset;
    double r1 = ShooterConstants.linkageLength;
    double r2 = ShooterConstants.driveArmLength;
    double theta =(((180+ShooterConstants.driverArmOffset)-(motorRot/ShooterConstants.wristGearRatio)*360))*UnitConstants.DEG_TO_RAD;
    double theta_deg = theta * UnitConstants.RAD_TO_DEG;
    // (((180+50.0177)-(motorRot/20.57)*360))*(Math.PI/180.0);
    double l1 = ShooterConstants.shooterConnectionLength;
    double[] P4 = {-r2* Math.cos(theta),-r2*Math.sin(theta)};
    double[] P1 = {6.75,0}; //its 6.75 right? //yes -matthew
    double d_14 = Math.sqrt(Math.pow((P1[0]-P4[0]),2)+Math.pow((P1[1]-P4[1]),2));
    double beta = Math.acos((Math.pow(d_14,2)+Math.pow(l1,2)
    -Math.pow(r1,2))/(2*d_14*l1));
    double d_45 = Math.sqrt(Math.pow(P4[0],2)+Math.pow(P4[1],2));
    double gamma = Math.acos((Math.pow(d_14,2)+Math.pow(P1[0],2)-Math.pow(d_45,2))/(2*d_14*P1[0]));
    double shotAngle = ((beta-gamma)*UnitConstants.RAD_TO_DEG)-ShooterConstants.shooterAngleToPivotAngle;
    if (theta_deg<180){
      shotAngle = ((beta+gamma)*UnitConstants.RAD_TO_DEG)-ShooterConstants.shooterAngleToPivotAngle;
    }
    return shotAngle;
  }

  // public double MotorRotToShootAngleMatthew(double motorRot){ //TODO: coded, needs testing
  //   //https://www.desmos.com/calculator/xdtyyrvaey
  //   double motorDriveAngle = motorRot * 360 / ShooterConstants.wristGearRatio - ShooterConstants.driverArmOffset;
  //   double[] basePoint = {
  //     6.75 - ShooterConstants.driveArmLength * Math.cos(motorDriveAngle*UnitConstants.DEG_TO_RAD),
  //     ShooterConstants.driveArmLength * Math.sin(motorDriveAngle*UnitConstants.DEG_TO_RAD)};
  //   double x1 = basePoint[0];
  //   double y1 = basePoint[1];
  //   double r1 = ShooterConstants.linkageLength;
  //   double r2 = ShooterConstants.shooterConnectionLength;
  //   double D = Math.sqrt(x1*x1+y1*y1);
  //   double[] topPoint = {
  //     x1 * (((r2*r2 - r1*r1) / (2*D*D)) + 0.5) 
  //     -y1*0.5*Math.sqrt(2*((r1*r1+r2*r2)/(D*D)) - ((Math.pow((r1*r1-r2*r2), 2))/(D*D*D*D)) - 1), 
  //     y1 * (((r2*r2 - r1*r1) / (2*D*D)) + 0.5) 
  //     +x1*0.5*Math.sqrt(2*((r1*r1+r2*r2)/(D*D)) - ((Math.pow((r1*r1-r2*r2), 2))/(D*D*D*D)) - 1), 
  //   };
  //   double atan2Deg =  Math.atan2(topPoint[1], topPoint[0]) * UnitConstants.RAD_TO_DEG;
  //   double atan2Normalized = (atan2Deg < 0)? atan2Deg+180 : atan2Deg-180;
  //   return atan2Normalized - ShooterConstants.shooterAngleToPivotAngle;
  // }
}
