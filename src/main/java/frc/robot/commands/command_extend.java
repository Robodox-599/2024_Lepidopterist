// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.subsystem_Rod;


public class command_extend extends Command {
/** Creates a new command_extend. */
private final subsystem_Rod m_rod;
private final DoubleSupplier desired;
public command_extend(subsystem_Rod rod, DoubleSupplier given) {
// Use addRequirements() here to declare subsystem dependencies.
m_rod = rod;
desired = given;
addRequirements(m_rod);
}


// Called when the command is initially scheduled.
@Override
public void initialize() {
m_rod.go(desired.getAsDouble());
}


// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {}


// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {}


// Returns true when the command should end.
@Override
public boolean isFinished() {
return false;
}
}



