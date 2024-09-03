package frc.robot.subsystems.intake.rollers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

// testing live share :O

public class Rollers extends SubsystemBase {
  private final RollersIO io;
  RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

  public Rollers(RollersIO io) {
    this.io = io;
    SmartDashboard.putData(getName(), this);
  }

  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Rollers", inputs);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setBrake(boolean brake) {
    io.setBrake(brake);
  }
  // public synchronized boolean spunUp() {
  // 	return topSpunUp() && bottomSpunUp();
  // }

  // /**
  //  * @return If both the top shooter motor is spun up past a minimum safe speed to avoid a jam,
  // and is within margin of the setpoint.
  //  */
  // public synchronized boolean topSpunUp() {
  // 	if (io.RollersIOInputs.topVelocityRadPerSec >= 300) {
  // 		boolean flywheelSpunUp = Util.epsilonEquals(
  // 				mPeriodicIO.top_demand, mPeriodicIO.top_velocity,
  // Constants.ShooterConstants.kFlywheelTolerance);
  // 		return flywheelSpunUp;
  // 	}
  // 	return false;
  // }

  // /**
  //  * @return If both the bottom shooter motor is spun up past a minimum safe speed to avoid a
  // jam, and is within margin of the setpoint.
  //  */
  // public synchronized boolean bottomSpunUp() {
  // 	if (mPeriodicIO.bottom_demand >= 300) {
  // 		boolean flywheelSpunUp = Util.epsilonEquals(
  // 				mPeriodicIO.bottom_demand,
  // 				mPeriodicIO.bottom_velocity,
  // 				Constants.ShooterConstants.kFlywheelTolerance);
  // 		return flywheelSpunUp;
  // 	}
  // 	return false;
  // }

  public Command setSpeed(double speed) {
    return Commands.run(
        () -> {
          io.setSpeed(speed);
        });
  }

  public Command speedCommand(DoubleSupplier speed) {
    return new FunctionalCommand(
        () -> {}, () -> io.setSpeed(speed.getAsDouble()), (stop) -> io.stop(), () -> false, this);
  }

  public Command manualCommand(DoubleSupplier voltage) {
    return new FunctionalCommand(
        () -> {},
        () -> io.setVoltage(voltage.getAsDouble()),
        (stop) -> io.stop(),
        () -> false,
        this);
  }

  public Command manualCommand(double voltage) {
    return manualCommand(() -> voltage);
  }

  public Command stop() {
    return new FunctionalCommand(
        () -> {}, () -> io.setVoltage(0), (stop) -> io.stop(), () -> false, this);
  }
}
