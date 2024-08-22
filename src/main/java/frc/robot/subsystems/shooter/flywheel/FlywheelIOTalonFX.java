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

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX topFlywheel = new TalonFX(flywheelTopMotorId);
  private final TalonFX bottomFlywheel = new TalonFX(flywheelTopMotorId);

  private final StatusSignal<Double> topFlywheelPosition = topFlywheel.getPosition();
  private final StatusSignal<Double> topFlywheelVelocity = topFlywheel.getVelocity();
  private final StatusSignal<Double> topFlywheelAppliedVolts = topFlywheel.getMotorVoltage();
  private final StatusSignal<Double> topFlywheelCurrent = topFlywheel.getSupplyCurrent();

  private final StatusSignal<Double> bottomFlywheelPosition = bottomFlywheel.getPosition();
  private final StatusSignal<Double> bottomFlywheelVelocity = bottomFlywheel.getVelocity();
  private final StatusSignal<Double> bottomFlywheelAppliedVolts = bottomFlywheel.getMotorVoltage();
  private final StatusSignal<Double> bottomFlywheelCurrent = bottomFlywheel.getSupplyCurrent();

  public FlywheelIOTalonFX() {
    var config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Slot0.kP = realFlywheelFeedBackkP;
    config.Slot0.kI = realFlywheelFeedBackkI;
    config.Slot0.kD = realFlywheelFeedBackkD;

    topFlywheel.getConfigurator().apply(config);
    bottomFlywheel.getConfigurator().apply(config);

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

    inputs.upperFlywheelPositionRad =
        Units.rotationsToRadians(topFlywheelPosition.getValueAsDouble()) / FLYWHEEL_GEAR_RATIO;
    inputs.upperFlywheelVelocityRadPerSec =
        Units.rotationsToRadians(topFlywheelVelocity.getValueAsDouble()) / FLYWHEEL_GEAR_RATIO;
    inputs.upperFlywheelAppliedVolts = topFlywheelAppliedVolts.getValueAsDouble();
    inputs.upperFlywheelCurrentAmps = topFlywheelCurrent.getValueAsDouble();

    inputs.lowerFlywheelPositionRad =
        Units.rotationsToRadians(bottomFlywheelPosition.getValueAsDouble()) / FLYWHEEL_GEAR_RATIO;
    inputs.lowerFlywheelVelocityRadPerSec =
        Units.rotationsToRadians(bottomFlywheelVelocity.getValueAsDouble()) / FLYWHEEL_GEAR_RATIO;
    inputs.lowerFlywheelAppliedVolts = bottomFlywheelAppliedVolts.getValueAsDouble();
    inputs.lowerFlywheelCurrentAmps = bottomFlywheelCurrent.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    topFlywheel.setControl(new VoltageOut(volts));
    bottomFlywheel.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocity(
      double topVelocityRadPerSec,
      double topFFVolts,
      double bottomVelocityRadPerSec,
      double bottomFFVolts) {
    topFlywheel.setControl(
        new VelocityDutyCycle(
            Units.radiansToRotations(topVelocityRadPerSec),
            0.0,
            false,
            topFFVolts,
            0,
            false,
            false,
            false));
    bottomFlywheel.setControl(
        new VelocityDutyCycle(
            Units.radiansToRotations(bottomVelocityRadPerSec),
            0.0,
            false,
            bottomFFVolts,
            0,
            false,
            false,
            false));
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
