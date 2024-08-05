package frc.robot.commands;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.intake.rollers.Rollers;
import frc.robot.subsystems.intake.wrist.IntakeWrist;

public class IntakeCommands extends Command {
  private DigitalInput beambreak = new DigitalInput(beamBreak1Port);

  public Command runIntakeuntilBeamBreak(Rollers rollers) {
    return Commands.sequence(
        rollers.speedCommand(() -> kSpeakerIntakeSpeed),
        new WaitUntilCommand(() -> !beambreak.get()),
        new WaitUntilCommand(extraIntakeTime),
        rollers.speedCommand(() -> 0));
  }

  public Command stopRollers(Rollers rollers) {
    return rollers.stop();
  }

  public Command runIntakeBackCMD(Rollers rollers) {
    return rollers.speedCommand(() -> kIntakeBackfeedSpeed).finallyDo(() -> stopRollers(rollers));
  }

  public Command runIntakeFwdCMD(Rollers rollers) {
    return rollers.speedCommand(() -> kSpeakerIntakeSpeed).finallyDo(() -> stopRollers(rollers));
  }

  public Command stowCommand(IntakeWrist wrist) {
    return Commands.sequence(wrist.PIDCommand(kWristRetractVal), wrist.PIDHoldCommand());
  }

  public Command extendCommand(IntakeWrist wrist) {
    return Commands.sequence(wrist.PIDCommand(kWristExtendVal), wrist.PIDHoldCommand());
  }

  public Command autoSpeakerIntake(IntakeWrist wrist, Rollers rollers) {
    return Commands.sequence(
        extendCommand(wrist), runIntakeuntilBeamBreak(rollers).finallyDo(() -> stowCommand(wrist)));
  }
}
