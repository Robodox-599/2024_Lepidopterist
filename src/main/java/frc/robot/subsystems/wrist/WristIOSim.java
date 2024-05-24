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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class WristIOSim implements WristIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private SingleJointedArmSim wristSim = new SingleJointedArmSim(DCMotor.getNEO(1), 4.0, 0.1/*fix*/, Units.inchesToMeters(13), Units.degreesToRadians(-50), Units.degreesToRadians(250), true, Units.degreesToRadians(-50));
  private FlywheelSim rollerSim = new FlywheelSim(DCMotor.getNEO(1), 1, 0.004);
  private PIDController rollerPID = new PIDController(1.0, 0.0, 0.0);
  private double rollerAppliedVolts = 0.0;
  private double rollerFFVolts = 0.0;

  private double wristAppliedVolts = 0.0;

  @Override
  public void updateInputs(WristIOInputs inputs) {
    //because we're doing rio pid, we don't needa do da math
    wristSim.update(LOOP_PERIOD_SECS);
    
    inputs.wristAngleRad = wristSim.getAngleRads();
    inputs.wristVelocityRadPerSec = wristSim.getVelocityRadPerSec();
    inputs.wristAppliedVolts = wristAppliedVolts;
    inputs.wristCurrentAmps = wristSim.getCurrentDrawAmps();
    inputs.wristTemperature = -1.0;
    
    //because we use motor pid, we do da math   
    rollerAppliedVolts =   
    MathUtil.clamp(rollerPID.calculate(rollerSim.getAngularVelocityRadPerSec())
         + rollerFFVolts, -12.0, 12.0);
    rollerSim.setInputVoltage(rollerAppliedVolts);
    rollerSim.update(LOOP_PERIOD_SECS);
    
    inputs.rollerVelocityRadPerSec = rollerSim.getAngularVelocityRadPerSec();
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerCurrentAmps = rollerSim.getCurrentDrawAmps();
    inputs.rollerTemperature = -1.0;    
  }

  @Override
  public void setWristVolts(double volts){
    wristAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    wristSim.setInputVoltage(wristAppliedVolts);
  }

  /** Run closed loop at the specified velocity. */
  @Override
  public void setRollerVelocity(double velocityRadPerSec, double ffVolts) {
    rollerPID.setSetpoint(velocityRadPerSec);
    rollerFFVolts = ffVolts;
  }

  /** Set velocity PID constants. */
  @Override
  public void configureRollerPID(double kP, double kI, double kD) {
    rollerPID.setPID(kP, kI, kD);
  }
}
