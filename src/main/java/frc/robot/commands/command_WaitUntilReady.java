// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.subsystem_ShooterWheels;
import frc.robot.subsystems.subsystem_ShooterWrist;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.subsystem_DriveTrain;

public class command_WaitUntilReady extends Command {
  /** Creates a new isReadyLEDs. */
  private final subsystem_ShooterWrist m_wrist;
  private final subsystem_DriveTrain m_drive;
  // private final subsystem_ShooterWheels m_wheels;

  public command_WaitUntilReady(/*subsystem_ShooterWheels wheels,*/ subsystem_ShooterWrist wrist, subsystem_DriveTrain drive) {
    // Use addRequirements() here to declare subsystem dependencies. //theres no requirements
    // m_wheels = wheels;
    m_wrist = wrist;
    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //add wrist + drive locking code
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (RobotConstants.robotColor==null){
      SmartDashboard.putBoolean("Alliance Color Error", true);
    }
    Translation2d speakerPos = (RobotConstants.robotColor==Alliance.Blue) ? 
    FieldConstants.blueSpeakerCenter.toTranslation2d():FieldConstants.redSpeakerCenter.toTranslation2d();
    if (speakerPos.getDistance(m_drive.getPose().getTranslation()) > ShooterConstants.shootRadius){
      return false;
    }
    if (!m_wrist.isWithinMargin(ShooterConstants.wristShootTolerance)){
      return false;
    }
    // if (!m_wheels.isWithinMargin(ShooterConstants.flywheelShootTolerance)){
    //   return false;
    // }
    return true;
  }
}
