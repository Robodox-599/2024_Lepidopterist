package frc.robot.commands;

import static frc.robot.commands.IndexerCommands.*;
import static frc.robot.subsystems.intake.Rollers.RollerConstants.*;
import static frc.robot.subsystems.intake.wrist.IntakeWristConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Rollers.Rollers;
import frc.robot.subsystems.intake.wrist.IntakeWrist;

public class IntakeCommands extends Command {
  public static Command runIntakeuntilBeamBreak(Rollers rollers) {
    return Commands.sequence(
        runIntakeFwdCMD(rollers), new WaitUntilCommand(extraIntakeTime), stopRollers(rollers));
  }

  public static Command stopRollers(Rollers rollers) {
    return rollers.stop();
  }

  public static Command runIntakeBackCMD(Rollers rollers) {
    return rollers.setSpeed(-0.4);
  }

  public static Command runIntakeFwdCMD(Rollers rollers) {
    return rollers.setSpeed(0.4);
  }

  public static Command stowCommand(IntakeWrist wrist) {
    return Commands.sequence(wrist.PIDCommand(() -> kWristRetractVal));
  }

  public static Command extendCommand(IntakeWrist wrist) {
    return Commands.sequence(wrist.PIDCommand(() -> kWristExtendVal));
  }

  public static Command intakeDeployAndIntake(IntakeWrist wrist, Rollers rollers, Indexer indexer) {
    return Commands.parallel(
        extendCommand(wrist), runIndexerUntilBeamBreak(indexer), runIntakeFwdCMD(rollers));
  }

  public static Command intakeStartEnd(
      IntakeWrist wrist, Rollers rollers, Indexer indexer, double time) {
    return new StartEndCommand(
        () -> intakeDeployAndIntake(wrist, rollers, indexer).andThen(new WaitUntilCommand(time)),
        () -> stowCommand(wrist));
  }
}
