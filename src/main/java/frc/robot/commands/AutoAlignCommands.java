package frc.robot.commands;

import static frc.robot.Constants.shouldFlip;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class AutoAlignCommands extends Command {
  public static double headingFlip() {
    if (shouldFlip()) {
      return (0);
    } else {
      return (180);
    }
  }

  public static Command autoAlignCommand(Drive drive) {
    return Commands.run(
        () -> {
          final PIDController angleController =
              new PIDController(autoTurnSpeakerkP, autoTurnSpeakerkI, autoTurnSpeakerkD);
          Pose2d speakerPose =
              AllianceFlipUtil.apply(new Pose2d(-0.2, (5 + 6.12) / 2, new Rotation2d(0)));
          angleController.setTolerance(0.08, 0.01);
          Transform2d targetTransform = drive.getPose().minus(speakerPose);
          Rotation2d targetDirection =
              new Rotation2d(targetTransform.getX(), targetTransform.getY())
                  .plus(new Rotation2d(Units.degreesToRadians(headingFlip())));
          ;
          Logger.recordOutput("Odometry/AutoAimDirection", targetDirection);
          angleController.setSetpoint(MathUtil.angleModulus(targetDirection.getRadians()));
          double omega =
              angleController.calculate(
                  MathUtil.angleModulus(drive.getRotation().getRadians()),
                  MathUtil.angleModulus(targetDirection.getRadians()));
          omega = Math.copySign(omega * omega, omega); // no idea why squared
          // Convert to robot relative speeds and send command
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  0, 0, omega * drive.getMaxAngularSpeedRadPerSec(), drive.getRotation()));
          if (angleController.atSetpoint()) {
            drive.stop();
            angleController.close();
          }
        },
        drive);
  }

  public static boolean pointedAtSpeaker(Drive drive) {
    Pose2d speakerPose =
        AllianceFlipUtil.apply(new Pose2d(-0.2, (5 + 6.12) / 2, new Rotation2d(0)));
    Transform2d targetTransform = drive.getPose().minus(speakerPose);
    Rotation2d targetDirection =
        new Rotation2d(targetTransform.getX(), targetTransform.getY()).plus(new Rotation2d(0));
    ;
    // Convert to robot relative speeds and send command
    if (Math.abs(drive.getRotation().getDegrees() - targetDirection.getDegrees())
        < autoAlignAngleThreshold) {
      return true;
    } else {
      return false;
    }
  }
}
