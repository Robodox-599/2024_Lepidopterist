package frc.robot.subsystems.intake.wrist;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeWrist extends SubsystemBase {
  private final IntakeWristIOInputsAutoLogged inputs = new IntakeWristIOInputsAutoLogged();
  private double setpoint = 0;

  private final IntakeWristIO io;

  // Create a Mechanism2d visualization of the arm
  private MechanismLigament2d wristMechanism = getArmMechanism();

  public IntakeWrist(IntakeWristIO io) {
    this.io = io;
    SmartDashboard.putData(getName(), this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeWrist", inputs);

    wristMechanism.setAngle(Units.radiansToDegrees(inputs.angleRads));

    // Log Inputs
    Logger.processInputs("IntakeWrist", inputs);
  }

  public void goPID() {
    io.goToSetpoint(setpoint);
  }

  public void setPID(double setpoint) {
    this.setpoint = setpoint;
    Logger.recordOutput("IntakeWrist/Setpoint", setpoint);
  }

  public boolean atSetpoint() {
    return Math.abs(io.getAngle() - setpoint) < IntakeWristConstants.intakeWristPositionTolerance
        && Math.abs(getVelocity()) < IntakeWristConstants.intakeWristVelocityTolerance;
  }

  public double getVelocity() {
    return inputs.angVelocityRadsPerSec;
  }

  public MechanismLigament2d getArmMechanism() {
    return new MechanismLigament2d("IntakeWrist", 0.4, 0, 5, new Color8Bit(Color.kAqua));
  }

  public Command PIDCommand(DoubleSupplier setpointSupplier) {
    return new FunctionalCommand(
        () -> setPID(setpointSupplier.getAsDouble()),
        () -> {
          Logger.recordOutput("IntakeWristAngle", setpointSupplier.getAsDouble());
          setPID(setpointSupplier.getAsDouble());
          goPID();
        },
        (stop) -> stop(),
        this::atSetpoint,
        this);
  }

  public Command stop() {
    return new FunctionalCommand(
        () -> {}, () -> io.setVoltage(0), (stop) -> io.stop(), () -> false, this);
  }
}
