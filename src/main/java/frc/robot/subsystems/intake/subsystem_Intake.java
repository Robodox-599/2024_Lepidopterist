// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.WristMotorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.HardwareConfig;
import java.util.function.DoubleSupplier;

public class subsystem_Intake extends SubsystemBase {

  private TalonFX m_IntakeMotor;
  private TalonFX m_WristMotor;
  private DigitalInput m_BeamBreak1;
  private DutyCycleEncoder m_AbsEncoder;
  private HardwareConfig m_Settings;

  // private double m_IntakeEnc;
  private double m_DesiredIntakePos;

  private double m_WristEnc; // where it at
  // private double m_WristExtend; //from in to out
  // private double m_WristRetract; //from out to in
  private double m_DesiredWristPos; // where wrist wants to go
  private double m_intakePercent;
  private int m_WristSlot;
  private Timer m_beamBreakTimer = new Timer();
  /** Creates a new subsystem_Intake. */
  public subsystem_Intake() {
    m_IntakeMotor = new TalonFX(IntakeConstants.RollerMotorConstants.intakeRollersMotorID, "rio");
    m_IntakeMotor.setNeutralMode(NeutralModeValue.Brake);

    m_WristMotor = new TalonFX(IntakeConstants.WristMotorConstants.wristMotorID, "rio");
    m_WristMotor.setNeutralMode(NeutralModeValue.Coast);

    m_BeamBreak1 = new DigitalInput(IntakeConstants.beamBreak1Port);
    m_AbsEncoder = new DutyCycleEncoder(IntakeConstants.throughBoreEncID);
    m_Settings = new HardwareConfig();

    m_IntakeMotor.getConfigurator().apply(m_Settings.getMotorConfig(0));
    m_IntakeMotor.setInverted(true);
    m_WristMotor.getConfigurator().apply(m_Settings.getMotorConfig(1));
    m_WristMotor.setNeutralMode(NeutralModeValue.Brake);

    m_DesiredIntakePos = 0.0;

    m_WristEnc = 0.0; // thrubore enc val
    m_DesiredWristPos = 0.0; // where wrist wants to go
    m_WristSlot = 0;

    resetWristPos();
    m_beamBreakTimer.start();
    m_IntakeMotor.setNeutralMode(NeutralModeValue.Brake);
    m_WristMotor.setPosition(0.0);
    SmartDashboard.putData("zero shooter wrist", zeroIntakeWrist());
  }

  public void stopIntake() {
    m_IntakeMotor.stopMotor();
  }

  public void resetWristPos() {
    m_AbsEncoder.reset();
  }

  public void setDesiredWristPos(double passedInPosition) {
    m_WristSlot =
        passedInPosition == IntakeConstants.kWristExtendVal
            ? WristMotorConstants.wristExtendSlot
            : WristMotorConstants.wristRetractSlot;
    m_DesiredWristPos = passedInPosition;
    MotionMagicVoltage m_request =
        new MotionMagicVoltage(m_DesiredWristPos)
            .withSlot(m_WristSlot)
            .withFeedForward(WristMotorConstants.kWristFeedForward);
    m_WristMotor.setControl(m_request.withPosition(m_DesiredWristPos));
  }

  public boolean isWristAtDesiredPosition() {
    SmartDashboard.putNumber(
        "diffsintake",
        Math.abs(m_WristMotor.getPosition().getValueAsDouble() - m_DesiredIntakePos));
    return Math.abs(m_WristMotor.getPosition().getValueAsDouble() - m_DesiredIntakePos)
        <= IntakeConstants.kWristTolerance;
  }

  public Command runIntakeuntilBeamBreak() {
    return Commands.sequence(
        runIntakeCommand(() -> IntakeConstants.kSpeakerIntakeSpeed),
        new WaitUntilCommand(() -> m_beamBreakTimer.get() >= IndexerConstants.beamBreakDebounce),
        new WaitUntilCommand(IntakeConstants.extraIntakeTime),
        new InstantCommand(() -> stopIntake(), this));
  }

  public Command moveWristCommand(DoubleSupplier passedInPosition) {
    return Commands.sequence(
        new InstantCommand(() -> setDesiredWristPos(passedInPosition.getAsDouble()), this),
        new WaitUntilCommand(
            () -> {
              return isWristAtDesiredPosition();
            }));
  }

  public Command stowCommand() {
    return Commands.sequence(
        new InstantCommand(() -> stopIntake(), this),
        new InstantCommand(() -> setDesiredWristPos(IntakeConstants.kWristRetractVal), this));
  }

  public Command runIntakeBackCMD() {
    return new StartEndCommand(
        () -> m_IntakeMotor.set(IntakeConstants.kIntakeBackfeedSpeed), () -> stopIntake(), this);
    // return new StartEndCommand(() -> runIntake(true, false, false), () -> stopIntake(), this);
  }

  public Command runIntakeFwdCMD() {
    return new StartEndCommand(
        () -> m_IntakeMotor.set(IntakeConstants.kSpeakerIntakeSpeed), () -> stopIntake(), this);
    // return runIntakeCommand(() -> IntakeConstants.kSpeakerIntakeSpeed).repeatedly().finallyDo(()
    // -> stopIntake());
  }

  public Command autonExtend() {
    // return runIntakeCommand(() -> IntakeConstants.kAutoIntakeSpeed).repeatedly().finallyDo(() ->
    // stopIntake());
    return new StartEndCommand(
        () -> m_IntakeMotor.set(IntakeConstants.kAutoIntakeSpeed), () -> stopIntake(), this);
  }

  public Command runIntakeCommand(DoubleSupplier intakePercent) {
    return new InstantCommand(() -> m_IntakeMotor.set(intakePercent.getAsDouble()), this);
  }

  /*
   * Runs intake until beam break 1, upon which it automatically stows
   */

  public Command autoSpeakerIntake() {
    return Commands.sequence(
        new InstantCommand(() -> setDesiredWristPos(IntakeConstants.kWristExtendVal), this),
        runIntakeCommand(() -> IntakeConstants.kSpeakerIntakeSpeed),
        new WaitUntilCommand(() -> m_beamBreakTimer.get() >= IndexerConstants.beamBreakDebounce),
        // new WaitUntilCommand(IntakeConstants.extraIntakeTime),
        stowCommand());
  }

  public Command autoAmpIntake() {
    return Commands.sequence(
        new InstantCommand(() -> setDesiredWristPos(IntakeConstants.kWristExtendVal), this),
        runIntakeCommand(() -> IntakeConstants.kAmpIntakeSpeed),
        new WaitUntilCommand(() -> m_beamBreakTimer.get() >= IndexerConstants.beamBreakDebounce),
        // new WaitUntilCommand(IntakeConstants.extraIntakeTime),
        stowCommand());
  }

  public Command autonIntake() {
    return Commands.sequence(
        new InstantCommand(() -> setDesiredWristPos(IntakeConstants.kWristExtendVal), this),
        runIntakeCommand(() -> IntakeConstants.kAutoIntakeSpeed),
        new WaitUntilCommand(() -> m_beamBreakTimer.get() >= IndexerConstants.beamBreakDebounce),
        // new WaitUntilCommand(IntakeConstants.extraIntakeTime),
        stowCommand());
  }

  public Command ampScoreCommand() {
    return Commands.sequence(
        new InstantCommand(() -> setDesiredWristPos(IntakeConstants.kWristAmpVal), this),
        new WaitUntilCommand(() -> isWristAtDesiredPosition()),
        Commands.waitSeconds(1),
        runIntakeCommand(() -> IntakeConstants.kIntakeAmpScoreSpeed));
  }

  public Command ampFling() {
    return Commands.sequence(
        new InstantCommand(() -> setDesiredWristPos(IntakeConstants.kWristAmpVal), this),
        new WaitUntilCommand(ShooterConstants.WristSepoints.ampScoringDelay),
        runIntakeCommand(() -> IntakeConstants.kIntakeAmpScoreSpeed));
  }

  public Command backwardsIntakeCommand() {
    return Commands.sequence(
        new InstantCommand(() -> setDesiredWristPos(IntakeConstants.kWristExtendVal), this),
        runIntakeCommand(() -> IntakeConstants.kIntakeBackfeedSpeed),
        new WaitUntilCommand(() -> m_beamBreakTimer.get() < IndexerConstants.beamBreakDebounce),
        new WaitUntilCommand(IntakeConstants.backwardsIntakeTime),
        new InstantCommand(() -> stopIntake(), this),
        new InstantCommand(() -> setDesiredWristPos(IntakeConstants.kWristRetractVal), this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_BeamBreak1.get()) {
      m_beamBreakTimer.restart();
    }

    // if(isWristAtDesiredPosition() && m_DesiredWristPos == IntakeConstants.kWristRetractVal){
    //   m_WristMotor.stopMotor(); //stop motor only if fully retracted
    // }

    SmartDashboard.putNumber("Intake Wrist Speed", m_WristMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Intake Voltage", m_WristMotor.getSupplyVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Intake Velocity", m_IntakeMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "Intake Wrist Thru Bore", m_AbsEncoder.getDistance() * IntakeConstants.wristGearRatio);
    SmartDashboard.putNumber(
        "Intake Wrist Built In", m_WristMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Current Slot", m_WristSlot);
    SmartDashboard.putBoolean("At intake Desired Wrist Position", isWristAtDesiredPosition());
    SmartDashboard.putNumber("Desired intake wrist pos", m_DesiredWristPos);
    SmartDashboard.putBoolean(
        "beambreak1", m_beamBreakTimer.get() >= IndexerConstants.beamBreakDebounce);
  }

  public Command zeroIntakeWrist() {
    return new InstantCommand(() -> m_WristMotor.setPosition(0.0)).ignoringDisable(true);
  }
}
