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

// meer is my cutie

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public double drivePositionMeters = 0.0;
    public double driveVelocityMetersPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};

    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double[] turnCurrentAmps = new double[] {};

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsMeters = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};

    public int index = 0;
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(final ModuleIOInputs inputs);

  /** Run the drive motor at the specified voltage. */
  public default void setDriveVoltage(final double volts) {
    setDriveVoltage(volts, true);
  }

  /** Run the drive motor at the specified voltage. */
  public void setDriveVoltage(final double volts, final boolean focEnabled);

  /** Use onboard PIDF to run the drive motor at the specified speed */
  public default void setDriveSetpoint(final double metersPerSecond) {
    setDriveSetpoint(metersPerSecond, 0.0);
  }

  /** Use onboard PIDF to run the drive motor at the specified speed */
  public void setDriveSetpoint(final double metersPerSecond, final double metersPerSecondSquared);

  /** Run the turn motor at the specified voltage. */
  public void setTurnVoltage(final double volts);

  /** Use onboard PIDF to run the turn motor to the specified rotation */
  public void setTurnSetpoint(final Rotation2d rotation);

  /** Gets the name of the swerve module for logging purposes, should be constant per-module. */
  public String getModuleName();
}
