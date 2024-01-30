// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.BreakerConstants;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class cGroup_Intake {

    public static Command intakeTime(subsystem_Wrist wrist, subsystem_Intake intake, subsystem_Breakers breakers){
        SmartDashboard.putBoolean("- cgruop entered", true);
        return Commands.sequence(
          new command_MoveWrist(wrist, () -> {return WristConstants.kWristExtendVal;}),
          new command_RunIntake(intake, breakers)
        );
    }

    // public static Command intakeRetract(subsystem_Intake wrist, subsystem_Intake intake){
    //     return Commands.sequence(
    //       new command_MoveWrist(wrist, () -> {return WristConstants.kWristRetractVal;})
    //     );
    // }




}
