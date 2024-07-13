// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */

// Class for interfacing with Talon FX motor controllers and CANcoders
// Each instance corresponds to one swerve module

public class ModuleIOTalonFX implements ModuleIO {

  // Drive and turn motor controllers also absolute encoder
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  // Timestampt to queue to track the time at which measurments are taken
  private final Queue<Double> timestampQueue;

  // Status signals for various measurements from drive motor
  private final StatusSignal<Double> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  // Status signals for various measurements from turn motor
  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;

  // Gear ratios for SDS MK4i L2, adjust as necessary // MATTHEW OR MEER PLEASE SETUP
  private final double DRIVE_GEAR_RATIO = DriveGearRatioConstant;
  private final double TURN_GEAR_RATIO = TurnGearRatioConstant;

  // Indicates if the turn motor is inverted
  private final boolean isTurnMotorInverted = true;
  // Offset angle for the CANcoder to calibrate to zero
  private final Rotation2d absoluteEncoderOffset;

  // Constructor initalizes motor controllers, encoders, and status signals
  public ModuleIOTalonFX(int index) {
    // Configuration of drive and turn Talon FX motors and CANcoder on module index
    switch (index) {
        // MODULE 0 HARDWARE IDENTIFIERS
      case 0:
        driveTalon = new TalonFX(Module0DriveTalon, canbus); // MEER AND MATTHEW CONFIGURE THIS
        turnTalon = new TalonFX(Module0TurnTalon, canbus); // MEER AND MATTHEW CONFIGURE THIS
        cancoder = new CANcoder(Module0Cancoder, canbus); // MEER AND MATTHEW CONFIGURE THIS
        absoluteEncoderOffset =
            new Rotation2d(
                Module0AbsoluteEncoderOffset); // 3.084 MUST BE CALIBRATED // MEER AND MATTHEW
        // CONFIGURE THIS
        break;

        // MODULE 1 HARDWARE IDENTIFIERS
      case 1:
        driveTalon = new TalonFX(Module1DriveTalon, canbus); // MEER AND MATTHEW CONFIGURE THIS
        turnTalon = new TalonFX(Module1TurnTalon, canbus); // MEER AND MATTHEW CONFIGURE THIS
        cancoder = new CANcoder(Module1Cancoder, canbus); // MEER AND MATTHEW CONFIGURE THIS
        absoluteEncoderOffset =
            new Rotation2d(
                Module1AbsoluteEncoderOffset); // -3.135 MUST BE CALIBRATED // MEER AND MATTHEW
        // CONFIGURE THIS
        break;

        // MODULE 2 HARDWARE IDENTIFIERS
      case 2:
        driveTalon = new TalonFX(Module2DriveTalon, canbus); // MEER AND MATTHEW CONFIGURE THIS
        turnTalon = new TalonFX(Module2TurnTalon, canbus); // MEER AND MATTHEW CONFIGURE THIS
        cancoder = new CANcoder(Module2Cancoder, canbus); // MEER AND MATTHEW CONFIGURE THIS
        absoluteEncoderOffset =
            new Rotation2d(
                Module2AbsoluteEncoderOffset); // -3.114 MUST BE CALIBRATED // MEER AND MATTHEW
        // CONFIGURE THIS
        break;

        // MODULE 3 HARDWARE IDENTIFIERS
      case 3:
        driveTalon = new TalonFX(Module3DriveTalon, canbus); // MEER AND MATTHEW CONFIGURE THIS
        turnTalon = new TalonFX(Module3TurnTalon, canbus); // MEER AND MATTHEW CONFIGURE THIS
        cancoder = new CANcoder(Module3Cancoder, canbus); // MEER AND MATTHEW CONFIGURE THIS
        absoluteEncoderOffset =
            new Rotation2d(
                Module3AbsoluteEncoderOffset); // 0.003 []\ MUST BE CALIBRATED // MEER AND
        // MATTHEW CONFIGURE THIS
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    // Configuration settings & current limits for drive motor
    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.SupplyCurrentLimit = DriveMotorSupplyCurrentLimitConstant;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    // Configuration settings & current limits for turn motor
    var turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.SupplyCurrentLimit = TurnMotorSupplyCurrentLimitConstant;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turnTalon.getConfigurator().apply(turnConfig);
    setTurnBrakeMode(true);

    // Apply default config to CANcoder
    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    // Initialize timestamp and position queues from odometry
    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getSupplyCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getSupplyCurrent();

    // Set update frequencies for status signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        Module.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    // optimize comms between Talons and CAN bus
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
  }

  // Method to update Inputs
  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnPosition.getValueAsDouble() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value / TURN_GEAR_RATIO))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        isTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Coast : NeutralModeValue.Coast;
    turnTalon.getConfigurator().apply(config);
  }
}
