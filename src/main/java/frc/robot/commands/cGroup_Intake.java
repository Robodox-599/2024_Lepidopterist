// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.subsystem_Indexer;
import frc.robot.subsystems.subsystem_Intake;

/** Add your docs here. */
public class cGroup_Intake {

    public static Command runIntakeSequence(subsystem_Intake intake, subsystem_Indexer indexer){
        SmartDashboard.putBoolean("- cgruop entered", true);
        return Commands.sequence(
          intake.moveWristCommand(() -> {return IntakeConstants.kWristExtendVal;}),
          intake.runIntakeCommand(),
          indexer.runIndexerCommand(),
          intake.moveWristCommand(() -> {return IntakeConstants.kWristRetractVal;})
        );
    }
}
