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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.LoggedTunableNumber;

public class FlywheelIOSim implements FlywheelIO {
  private DCMotorSim simBottom = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 1.25, 0.004);
  private DCMotorSim simTop = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 1.25, 0.004);
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVoltsTop = 0.0;
  private double ffVoltsBottom = 0.0;
  private double appliedVoltsBottom = 0.0;
  private double appliedVoltsTop = 0.0;
  private static LoggedTunableNumber inputNoiseRPM =
      new LoggedTunableNumber("Flywheel Noise Input", 0);

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (closedLoop) {
      appliedVoltsBottom =
          MathUtil.clamp(
              pid.calculate(simBottom.getAngularVelocityRadPerSec()) + ffVoltsBottom, -12.0, 12.0);
      simBottom.setInputVoltage(appliedVoltsBottom);

      appliedVoltsTop =
          MathUtil.clamp(
              pid.calculate(simTop.getAngularVelocityRadPerSec()) + ffVoltsTop, -12.0, 12.0);
      simTop.setInputVoltage(appliedVoltsTop);
    }
    simTop.update(0.02);
    simBottom.update(0.02);

    inputs.positionRad = new double[] {0.0};
    inputs.velocityRadPerSec =
        new double[] {
          simTop.getAngularVelocityRadPerSec(),
        };
    inputs.appliedVolts = new double[] {appliedVoltsTop, appliedVoltsBottom};
    inputs.currentAmps = new double[] {simTop.getCurrentDrawAmps(), simBottom.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVoltsTop = volts;
    simTop.setInputVoltage(volts);
    appliedVoltsBottom = volts;
    simBottom.setInputVoltage(volts);
  }

  public double errorCalc(double inputVelocityRadPerSec, double target) {
    double currentRPM = inputVelocityRadPerSec * 60;
    double targetRPM = Units.radiansToRotations(target) * 60; // Convert to RPM
    double error = targetRPM - currentRPM;
    return error;
  }

  @Override
  public void setVelocity(
      double topVelocityRadPerSec,
      double topFFVolts,
      double bottomVelocityRadPerSec,
      double bottomFFVolts) {
    closedLoop = true;
    if (Math.abs(
            errorCalc(
                simTop.getAngularVelocityRadPerSec() + inputNoiseRPM.getAsDouble(),
                topVelocityRadPerSec))
        > acceptableErrorRPM) {
      pid.setSetpoint(topVelocityRadPerSec);
      this.ffVoltsTop = topFFVolts;
    }
    if (Math.abs(
            errorCalc(
                simBottom.getAngularVelocityRadPerSec() + inputNoiseRPM.getAsDouble(),
                bottomVelocityRadPerSec))
        > acceptableErrorRPM) {
      pid.setSetpoint(bottomVelocityRadPerSec);
      this.ffVoltsBottom = topFFVolts;
    }
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
