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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private double topGoalVelocityRPS = 0;
  private double bottomGoalVelocityRPS = 0;
  private double motorVoltage = 0;
  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
    Logger.recordOutput("Flywheel/topGoalVelocityRPM", topGoalVelocityRPS);
    Logger.recordOutput("Flywheel/bottomGoalVelocityRPM", bottomGoalVelocityRPS);
  }

  @AutoLogOutput
  public boolean flywheelsSpunUp() {
    return bottomFlywheelSpunUp(bottomGoalVelocityRPS) && topFlywheelSpunUp(topGoalVelocityRPS);
  }

  private boolean topFlywheelSpunUp(double goalVelocity) {
    return inDeadBand(getTopVelocityRPM(), (goalVelocity));
  }

  private boolean bottomFlywheelSpunUp(double goalVelocity) {
    return inDeadBand(getBottomVelocityRPM(), goalVelocity);
  }

  private boolean inDeadBand(double currentVelocityRPS, double goalVelocityRPS) {
    double targetRPS = goalVelocityRPS;
    double error = targetRPS - currentVelocityRPS;
    Logger.recordOutput("Flywheel/ErrorRPS", Math.abs(error));
    return (!(Math.abs(error) > 0.5));
  }
  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double topVelocityRPS, double bottomVelocityRPS) {
    io.setVelocity(topVelocityRPS, topVelocityRPS);
    topGoalVelocityRPS = topVelocityRPS;
    bottomGoalVelocityRPS = bottomVelocityRPS;
  }

  public void setSpeeds(double speed) {
    io.setSpeed(speed);
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }
  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getTopVelocityRPM() {
    return Units.radiansToRotations(inputs.upperFlywheelVelocityRadPerSec);
  }

  @AutoLogOutput
  public double getBottomVelocityRPM() {
    return Units.radiansToRotations(inputs.lowerFlywheelVelocityRadPerSec);
  }

  public void setVoltage(double volts) {
    setVoltage(() -> volts);
  }

  public Command runVoltage(DoubleSupplier volts) {
    return new FunctionalCommand(
        () -> setVoltage(volts), // no PID for now
        () -> {
          setVoltage(volts);
        },
        (interrupted) -> {
          if (interrupted) {
            io.stop();
          }
        },
        () -> false,
        this);
  }

  public void setVoltage(DoubleSupplier motorVoltageSup) {
    motorVoltage = motorVoltageSup.getAsDouble() * 10;
    io.setVoltage(motorVoltage);
  }

  public Command runVoltageBoth(DoubleSupplier volts) {
    return new FunctionalCommand(
        () -> setVoltage(volts), // no PID for now
        () -> {
          setVoltage(volts);
        },
        (interrupted) -> {
          if (interrupted) {
            io.stop();
          }
        },
        () -> false,
        this);
  }

  public Command runVoltage(double volts) {
    return runVoltage(() -> volts);
  }

  public Command runFlywheelVelocity(double topVelo, double bottomVelo) {
    return new FunctionalCommand(
        () -> runVelocity(topVelo, bottomVelo),
        () -> {
          runVelocity(topVelo, bottomVelo);
        },
        (interrupted) -> {
          if (interrupted) {
            io.stop();
          }
        },
        () -> flywheelsSpunUp(),
        this);
  }

  public Command setSpeed(double speed) {
    return new FunctionalCommand(
        () -> setSpeeds(speed),
        () -> {
          setSpeeds(speed);
        },
        (interrupted) -> {
          if (interrupted) {
            io.stop();
          }
        },
        () -> false,
        this);
  }
}
