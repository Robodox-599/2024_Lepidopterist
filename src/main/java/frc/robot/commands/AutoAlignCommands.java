package frc.robot.commands;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.*;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class AutoAlignCommands {

  private static PIDController angleController =
      new PIDController(autoTurnSpeakerkP, autoTurnSpeakerkI, autoTurnSpeakerkD);

  private static double speakerFlip() {
    // Logger.recordOutput("Odometry/AutoAlignColor", shouldFlip());
    if (RobotConstants.robotColor == Alliance.Red) {
      // Logger.recordOutput("Odometry/AutoAlignColor", shouldFlip());
      // Logger.recordOutput("Odometry/AutoAlignColor", isred);
      Logger.recordOutput("Odometry/AutoAlignColor", true);
      return (Units.inchesToMeters(651.223) - 0.2);
    } else {
      Logger.recordOutput("Odometry/AutoAlignColor", false);
      return (-0.2);
    }
    // try {
    //   Optional<Alliance> ally = DriverStation.getAlliance();
    //   if (ally.get() == Alliance.Red) {
    //     return (Units.inchesToMeters(651.223) - 0.2);
    //   } else {
    //     return (-0.2);
    //   }
    // } catch (NoSuchElementException e) {
    //   speakerFlip();
    // }
    // return 0;
  }

  public static Command turnSpeakerAngle(Drive drive) {
    Pose2d speakerPose = new Pose2d(speakerFlip(), ((5 + 6.12) / 2), new Rotation2d(0));
    Logger.recordOutput("Odometry/AutoAimSpeakerPose", speakerFlip());
    angleController.setTolerance(0.08, 0.01);
    return new FunctionalCommand(
        () -> {
          Transform2d targetTransform = drive.getPose().minus(speakerPose);
          Rotation2d targetDirection =
              new Rotation2d(targetTransform.getX(), targetTransform.getY())
                  .plus(new Rotation2d(Units.degreesToRadians(180)));
          angleController.setSetpoint(MathUtil.angleModulus(targetDirection.getRadians()));
        },
        () -> {
          // defines distance from speaker
          Transform2d targetTransform = drive.getPose().minus(speakerPose);
          Rotation2d targetDirection =
              new Rotation2d(targetTransform.getX(), targetTransform.getY())
                  .plus(new Rotation2d(Units.degreesToRadians(180)));
          ;
          Logger.recordOutput("Odometry/AutoAimDirection", targetDirection);

          double omega =
              angleController.calculate(
                  MathUtil.angleModulus(drive.getRotation().getRadians()),
                  MathUtil.angleModulus(targetDirection.getRadians()));
          omega = Math.copySign(omega * omega, omega); // no idea why squared
          // Convert to robot relative speeds and send command
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  0, 0, omega * drive.getMaxAngularSpeedRadPerSec(), drive.getRotation()));
        },
        (interrupted) -> {
          drive.stop();
        },
        () -> angleController.atSetpoint(),
        drive);
  }

  // public static boolean pointedAtSpeaker(Drive drive) {
  //   Pose2d speakerPose = FieldConstants.SpeakerPosition;
  //   Transform2d targetTransform = drive.getPose().minus(speakerPose);
  //   Rotation2d targetDirection =
  //       new Rotation2d(targetTransform.getX(), targetTransform.getY()).plus(new Rotation2d(0));
  //   ;

  //   // Convert to robot relative speeds and send command
  //   if (Math.abs(drive.getRotation().getDegrees() - targetDirection.getDegrees())
  //       < DriveConstants.autoAlignAngleThreshold) {
  //     return true;
  //   } else {
  //     return false;
  //   }
  // }
}
