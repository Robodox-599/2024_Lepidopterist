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

import static frc.robot.Constants.*;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
  private SimpleMotorFeedforward ffModel;
  private double topGoalVelocityRPM = 0;
  private double bottomGoalVelocityRPM = 0;
  private double motorVoltage = 0;
  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)

    switch (robotType) {
      case REALBOT:
        ffModel =
            new SimpleMotorFeedforward(
                realFlywheelFeedForwardkS, realFlywheelFeedForwardkV, realFlywheelFeedForwardkA);
        io.configurePID(realFlywheelFeedBackkP, realFlywheelFeedBackkI, realFlywheelFeedBackkD);

        break;
      case REPLAYBOT:
        ffModel =
            new SimpleMotorFeedforward(
                simFlywheelFeedForwardkS, simFlywheelFeedForwardkV, simFlywheelFeedForwardkA);
        io.configurePID(simFlywheelFeedBackkP, simFlywheelFeedBackkI, simFlywheelFeedBackkD);

        break;
      case SIMBOT:
        ffModel =
            new SimpleMotorFeedforward(
                simFlywheelFeedForwardkS, simFlywheelFeedForwardkV, simFlywheelFeedForwardkA);
        io.configurePID(simFlywheelFeedBackkP, simFlywheelFeedBackkI, simFlywheelFeedBackkD);

        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
    Logger.recordOutput("Flywheel/topGoalVelocityRPM", topGoalVelocityRPM);
    Logger.recordOutput("Flywheel/bottomGoalVelocityRPM", bottomGoalVelocityRPM);
  }

  @AutoLogOutput
  public boolean flywheelsSpunUp() {
    return bottomFlywheelSpunUp(bottomGoalVelocityRPM) && topFlywheelSpunUp(topGoalVelocityRPM);
  }

  private boolean topFlywheelSpunUp(double goalVelocity) {
    return inDeadBand(
        getTopVelocityRPM(), Units.rotationsPerMinuteToRadiansPerSecond(goalVelocity));
  }

  private boolean bottomFlywheelSpunUp(double goalVelocity) {
    return inDeadBand(
        getBottomVelocityRPM(), Units.rotationsPerMinuteToRadiansPerSecond(goalVelocity));
  }

  private boolean inDeadBand(double currentVelocityRPM, double goalVelocityRadPerSec) {
    double targetRPM = Units.radiansPerSecondToRotationsPerMinute(goalVelocityRadPerSec);
    double error = targetRPM - currentVelocityRPM;
    Logger.recordOutput("Flywheel/ErrorRPM", Math.abs(error));
    return (!(Math.abs(error) > acceptableErrorRPM));
  }
  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double topVelocityRPM, double bottomVelocityRPM) {
    var topVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(topVelocityRPM);
    var bottomVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(bottomVelocityRPM);
    io.setVelocity(
        topVelocityRadPerSec,
        ffModel.calculate(topVelocityRadPerSec),
        bottomVelocityRadPerSec,
        ffModel.calculate(bottomVelocityRadPerSec));
    topGoalVelocityRPM = topVelocityRPM;
    bottomGoalVelocityRPM = bottomVelocityRPM;
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }
  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getTopVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.upperFlywheelVelocityRadPerSec)
        * FLYWHEEL_GEAR_RATIO;
  }

  @AutoLogOutput
  public double getBottomVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.lowerFlywheelVelocityRadPerSec)
        * FLYWHEEL_GEAR_RATIO;
  }

  public void setVoltage(double volts) {
    setVoltage(() -> volts);
  }

  /** Returns the current velocity in radians per second. */
  public double getTopCharacterizationVelocity() {
    return inputs.upperFlywheelVelocityRadPerSec;
  }

  public double getBottomCharacterizationVelocity() {
    return inputs.lowerFlywheelVelocityRadPerSec;
  }

  public Command runVoltage(DoubleSupplier volts) {
    return new FunctionalCommand(
        // () -> setRPM(speed.getAsDouble(), speed.getAsDouble()),
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
        () -> false,
        this);
  }
}
