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

package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;

public class WristIOSparkMax implements WristIO {
  private static final double GEAR_RATIO = 1;

  private final CANSparkMax wrist = new CANSparkMax(0, MotorType.kBrushless);
  private final RelativeEncoder wristEncoder = wrist.getEncoder();
  
  private final CANSparkMax rollers = new CANSparkMax(1, MotorType.kBrushless);
  private final RelativeEncoder rollerEncoder = rollers.getEncoder();
  private final SparkPIDController rollerPID = rollers.getPIDController();

  public WristIOSparkMax() {
    wrist.restoreFactoryDefaults();
    rollers.restoreFactoryDefaults();

    wrist.setCANTimeout(250);
    rollers.setCANTimeout(250);

    wrist.setInverted(false);
    rollers.setInverted(false);

    wrist.enableVoltageCompensation(12.0);
    rollers.enableVoltageCompensation(12.0);

    wrist.setSmartCurrentLimit(25);
    rollers.setSmartCurrentLimit(30);

    wrist.burnFlash();
    rollers.burnFlash();
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.wristAngleRad = Units.rotationsToRadians(wristEncoder.getPosition() / GEAR_RATIO);
    inputs.wristVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(wristEncoder.getVelocity() / GEAR_RATIO);
    inputs.wristAppliedVolts = wrist.getAppliedOutput() * wrist.getBusVoltage();
    inputs.wristCurrentAmps = wrist.getOutputCurrent();
    inputs.wristTemperature = wrist.getMotorTemperature();

    inputs.rollerVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(rollerEncoder.getVelocity() / GEAR_RATIO);
    inputs.rollerAppliedVolts = rollers.getAppliedOutput() * rollers.getBusVoltage();
    inputs.rollerCurrentAmps = rollers.getOutputCurrent();
    inputs.rollerTemperature = rollers.getMotorTemperature(); 
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }
}
