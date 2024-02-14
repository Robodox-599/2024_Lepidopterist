// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.subsystem_Breakers;
import frc.robot.subsystems.subsystem_Indexer;
import frc.robot.subsystems.subsystem_Intake;
import frc.robot.subsystems.subsystem_Wrist;

/** Add your docs here. */
public class cGroup_Intake {

    public static Command intakeTime(subsystem_Wrist wrist, subsystem_Intake intake, subsystem_Breakers breakers, subsystem_Indexer indexer){
        SmartDashboard.putBoolean("- cgruop entered", true);
        return Commands.sequence(
          new command_MoveWrist(wrist, () -> {return WristConstants.kWristExtendVal;}),
          new command_RunIntake(intake, breakers),
          new command_RunIndexer(indexer, breakers),
          new command_MoveWrist(wrist, () -> {return WristConstants.kWristRetractVal;})
        );
    }
}
