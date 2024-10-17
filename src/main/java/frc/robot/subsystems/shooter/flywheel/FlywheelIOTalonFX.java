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

package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.flywheelBotomMotorId;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.flywheelTopMotorId;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.realBottomFlywheelFeedBackkD;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.realBottomFlywheelFeedBackkI;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.realBottomFlywheelFeedBackkP;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.realBottomFlywheelFeedForwardkS;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.realBottomFlywheelFeedForwardkV;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.realTopFlywheelFeedBackkD;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.realTopFlywheelFeedBackkI;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.realTopFlywheelFeedBackkP;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.realTopFlywheelFeedForwardkS;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.realTopFlywheelFeedForwardkV;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX topFlywheel = new TalonFX(flywheelTopMotorId, "rio");
  private final TalonFX bottomFlywheel = new TalonFX(flywheelBotomMotorId, "rio");

  private final StatusSignal<Double> topFlywheelPosition = topFlywheel.getPosition();
  private final StatusSignal<Double> topFlywheelVelocity = topFlywheel.getVelocity();
  private final StatusSignal<Double> topFlywheelAppliedVolts = topFlywheel.getMotorVoltage();
  private final StatusSignal<Double> topFlywheelCurrent = topFlywheel.getSupplyCurrent();

  private final StatusSignal<Double> bottomFlywheelPosition = bottomFlywheel.getPosition();
  private final StatusSignal<Double> bottomFlywheelVelocity = bottomFlywheel.getVelocity();
  private final StatusSignal<Double> bottomFlywheelAppliedVolts = bottomFlywheel.getMotorVoltage();
  private final StatusSignal<Double> bottomFlywheelCurrent = bottomFlywheel.getSupplyCurrent();

  public FlywheelIOTalonFX() {
    var configUpper = new TalonFXConfiguration();
    var configLower = new TalonFXConfiguration();

    // upper flywheel config
    configUpper.Slot0.kP = realTopFlywheelFeedBackkP;
    configUpper.Slot0.kI = realTopFlywheelFeedBackkI;
    configUpper.Slot0.kD = realTopFlywheelFeedBackkD;

    configUpper.Slot0.kS = realTopFlywheelFeedForwardkS;
    configUpper.Slot0.kV = realTopFlywheelFeedForwardkV;

    configUpper.CurrentLimits.SupplyCurrentLimit = 40.0;
    configUpper.CurrentLimits.SupplyCurrentLimitEnable = true;

    configUpper.CurrentLimits.StatorCurrentLimit = 45.0;
    configUpper.CurrentLimits.StatorCurrentLimitEnable = true;

    configUpper.MotorOutput.PeakReverseDutyCycle = 0.0;
    configUpper.Voltage.PeakReverseVoltage = 0.0;
    configUpper.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;

    configUpper.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    configUpper.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // lower flywheel config
    configLower.Slot0.kP = realBottomFlywheelFeedBackkP;
    configLower.Slot0.kI = realBottomFlywheelFeedBackkI;
    configLower.Slot0.kD = realBottomFlywheelFeedBackkD;

    configLower.Slot0.kS = realBottomFlywheelFeedForwardkS;
    configLower.Slot0.kV = realBottomFlywheelFeedForwardkV;

    configLower.CurrentLimits.SupplyCurrentLimit = 40.0;
    configLower.CurrentLimits.SupplyCurrentLimitEnable = true;

    configLower.CurrentLimits.StatorCurrentLimit = 45.0;
    configLower.CurrentLimits.StatorCurrentLimitEnable = true;

    configLower.MotorOutput.PeakReverseDutyCycle = 0.0;
    configLower.Voltage.PeakReverseVoltage = 0.0;
    configLower.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;

    configLower.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    topFlywheel.getConfigurator().apply(configUpper);
    bottomFlywheel.getConfigurator().apply(configLower);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        topFlywheelPosition,
        topFlywheelVelocity,
        topFlywheelAppliedVolts,
        topFlywheelCurrent,
        bottomFlywheelPosition,
        bottomFlywheelVelocity,
        bottomFlywheelAppliedVolts,
        bottomFlywheelCurrent);
    topFlywheel.optimizeBusUtilization();
    bottomFlywheel.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        topFlywheelPosition,
        topFlywheelVelocity,
        topFlywheelAppliedVolts,
        topFlywheelCurrent,
        bottomFlywheelPosition,
        bottomFlywheelVelocity,
        bottomFlywheelAppliedVolts,
        bottomFlywheelCurrent);

    inputs.upperFlywheelPositionRad =
        Units.rotationsToRadians(topFlywheelPosition.getValueAsDouble());
    inputs.upperFlywheelVelocityRadPerSec =
        Units.rotationsToRadians(topFlywheelVelocity.getValueAsDouble());
    inputs.upperFlywheelAppliedVolts = topFlywheelAppliedVolts.getValueAsDouble();
    inputs.upperFlywheelCurrentAmps = topFlywheelCurrent.getValueAsDouble();

    inputs.lowerFlywheelPositionRad =
        Units.rotationsToRadians(bottomFlywheelPosition.getValueAsDouble());
    inputs.lowerFlywheelVelocityRadPerSec =
        Units.rotationsToRadians(bottomFlywheelVelocity.getValueAsDouble());
    inputs.lowerFlywheelAppliedVolts = bottomFlywheelAppliedVolts.getValueAsDouble();
    inputs.lowerFlywheelCurrentAmps = bottomFlywheelCurrent.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    topFlywheel.setControl(new VoltageOut(-volts));
    bottomFlywheel.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocity(double topVelocityRadPerSec, double bottomVelocityRadPerSec) {
    bottomFlywheel.setControl(new VelocityVoltage(bottomVelocityRadPerSec));
    topFlywheel.setControl(new VelocityVoltage(topVelocityRadPerSec));
  }

  @Override
  public void stop() {
    topFlywheel.stopMotor();
    bottomFlywheel.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    topFlywheel.getConfigurator().apply(config);
    bottomFlywheel.getConfigurator().apply(config);
  }
}
