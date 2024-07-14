package frc.robot.commands;

import static frc.robot.Constants.shouldFlip;
import static frc.robot.FieldConstants.wingX;
import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.CommandXboxControllerSubsystem.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class AutoAlignCommands extends Command {
  // Determines if the heading should be flipped based on the alliance
  public static double headingFlip() {
    if (shouldFlip()) {
      return (0); // No flip needed, return 0 degrees
    } else {
      return (180); // Flip needed, return 180 degrees
    }
  }

  public static Command rumbleControllers(CommandXboxController driver) {
    // wtf
    return new StartEndCommand(
        () -> driver.getHID().setRumble(RumbleType.kBothRumble, 1),
        () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0));
  }

  public static Command autoAlignCommand(Drive drive, CommandXboxController controller) {
    return Commands.run(
        () -> {
          // Get the position of the speaker on the field, adjusted for alliance
          Pose2d speakerPose =
              AllianceFlipUtil.apply(new Pose2d(-0.2, (5 + 6.12) / 2, new Rotation2d(0)));
          if (drive.getPose().minus(speakerPose).getX() > wingX) {
            new StartEndCommand(
                () -> controller.getHID().setRumble(RumbleType.kBothRumble, 1),
                () -> controller.getHID().setRumble(RumbleType.kBothRumble, 0));
          } else {

            // Create a new PID controller for controlling the angle
            final PIDController angleController =
                new PIDController(autoTurnSpeakerkP, autoTurnSpeakerkI, autoTurnSpeakerkD);

            // Set tolerance for the PID controller
            angleController.setTolerance(0.08, 0.01);
            // Calculate the transformation from the robot's pose to the speaker's pose
            Transform2d targetTransform = drive.getPose().minus(speakerPose);
            // Calculate the target direction with the possible heading flip
            Rotation2d targetDirection =
                new Rotation2d(targetTransform.getX(), targetTransform.getY())
                    .plus(new Rotation2d(Units.degreesToRadians(headingFlip())));
            ;
            // Log targetDirection to easily identify if it is correct
            Logger.recordOutput("Odometry/AutoAimDirection", targetDirection);
            // Set the PID controller's setpoint to the target direction
            angleController.setSetpoint(MathUtil.angleModulus(targetDirection.getRadians()));
            // Calculate the angular velocity to align with the target direction
            double omega =
                angleController.calculate(
                    MathUtil.angleModulus(drive.getRotation().getRadians()),
                    MathUtil.angleModulus(targetDirection.getRadians()));
            omega = Math.copySign(omega * omega, omega); // Square the value and preserve the sign
            // Command the drive to run at the calculated angular velocity
            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    0, 0, omega * drive.getMaxAngularSpeedRadPerSec(), drive.getRotation()));
            // If the robot is aligned, stop the drive and close the PID controller
            if (angleController.atSetpoint()) {
              drive.stop();
              angleController.close();
            }
          }
        },
        drive);
  }

  public static boolean pointedAtSpeaker(Drive drive) {
    // Get the position of the speaker on the field, adjusted for alliance
    Pose2d speakerPose =
        AllianceFlipUtil.apply(new Pose2d(-0.2, (5 + 6.12) / 2, new Rotation2d(0)));
    // Calculate the transformation from the robot's pose to the speaker's pose
    Transform2d targetTransform = drive.getPose().minus(speakerPose);
    // Calculate the target direction considering heading flip
    Rotation2d targetDirection =
        new Rotation2d(targetTransform.getX(), targetTransform.getY())
            .plus(new Rotation2d(headingFlip()));
    ;
    // Check if the robot's current rotation is within the alignment threshold
    if (Math.abs(drive.getRotation().getDegrees() - targetDirection.getDegrees())
        < autoAlignAngleThreshold) {
      return true; // Robot is pointed at the speaker
    } else {
      return false; // Robot is not pointed at the speaker
    }
  }
}
