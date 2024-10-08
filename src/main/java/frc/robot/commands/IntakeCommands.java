package frc.robot.commands;

import static frc.robot.commands.IndexerCommands.*;
import static frc.robot.subsystems.intake.rollers.RollerConstants.*;
import static frc.robot.subsystems.intake.wrist.IntakeWristConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.intake.rollers.Rollers;
import frc.robot.subsystems.intake.wrist.IntakeWrist;

public class IntakeCommands extends Command {
  public static Command intakeDeployAndIntake(IntakeWrist intake, Rollers rollers) {
    return Commands.sequence(
        extendCommand(intake),
        runIntakeFwdCMD(rollers),
        new WaitUntilCommand(() -> rollers.getBeamBreak()),
        // new WaitUntilCommand(IntakeConstants.extraIntakeTime),
        Commands.parallel(stowCommand(intake), stopRollers(rollers)));
  }

  public static Command stopRollers(Rollers rollers) {
    return rollers.stop();
  }

  public static Command runIntakeBackCMD(Rollers rollers) {
    return rollers.setSpeed(-0.4);
  }

  public static Command runIntakeFwdCMD(Rollers rollers) {
    return rollers.setSpeed(0.9);
  }

  public static Command stowCommand(IntakeWrist wrist) {
    return Commands.sequence(wrist.PIDCommand(() -> kWristRetractVal));
  }

  public static Command extendCommand(IntakeWrist wrist) {
    return Commands.sequence(wrist.PIDCommand(() -> kWristExtendVal));
  }

  // public static Command intakeDeployAndIntake(IntakeWrist wrist, Rollers rollers, Indexer
  // indexer) {
  //   return Commands.parallel(
  //       extendCommand(wrist), indexer.runIndexerBeamBreak(), runIntakeFwdCMD(rollers));
  // }
}
