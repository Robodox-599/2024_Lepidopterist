package frc.robot.commands;

import static frc.robot.commands.IndexerCommands.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.rollers.Rollers;
import frc.robot.subsystems.intake.wrist.IntakeWrist;

public class IntakeCommands extends Command {
  private static DigitalInput beambreak = new DigitalInput(beamBreak1Port);

  public static Command runIntakeuntilBeamBreak(Rollers rollers) {
    return Commands.sequence(
        rollers.speedCommand(() -> kSpeakerIntakeSpeed),
        new WaitUntilCommand(() -> !beambreak.get()),
        new WaitUntilCommand(extraIntakeTime),
        rollers.speedCommand(() -> 0));
  }

  public static Command stopRollers(Rollers rollers) {
    return rollers.stop();
  }

  public static Command runIntakeBackCMD(Rollers rollers) {
    return rollers.speedCommand(() -> kIntakeBackfeedSpeed).finallyDo(() -> stopRollers(rollers));
  }

  public static Command runIntakeFwdCMD(Rollers rollers) {
    return rollers.speedCommand(() -> kSpeakerIntakeSpeed).finallyDo(() -> stopRollers(rollers));
  }

  public static Command stowCommand(IntakeWrist wrist) {
    return Commands.sequence(wrist.PIDCommand(kWristRetractVal), wrist.PIDHoldCommand());
  }

  public static Command extendCommand(IntakeWrist wrist) {
    return Commands.sequence(wrist.PIDCommand(kWristExtendVal), wrist.PIDHoldCommand());
  }

  public static Command intakeDeployAndIntake(IntakeWrist wrist, Rollers rollers, Indexer indexer) {
    return Commands.sequence(
        extendCommand(wrist), runIndexerUntilBeamBreak(indexer), runIntakeuntilBeamBreak(rollers));
  }

  public static Command intakeStartEnd(
      IntakeWrist wrist, Rollers rollers, Indexer indexer, double time) {
    return new StartEndCommand(
        () -> intakeDeployAndIntake(wrist, rollers, indexer).andThen(new WaitUntilCommand(time)),
        () -> stowCommand(wrist));
  }
}
