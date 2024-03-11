// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.Constants.ShooterConstants;
// import frc.robot.subsystems.subsystem_Shooter;

// public class command_ToSetWrist extends Command {
//   /** Creates a new command_ToAmpWrist. */
//   private final subsystem_Shooter m_Shooter;
//   private final DoubleSupplier m_setpoint;
//   private final Timer m_WristTimer = new Timer();
//   private final Timer m_WheelTimer = new Timer();

//   public command_ToSetWrist(subsystem_Shooter shooter, DoubleSupplier setpoint) {
//     m_Shooter = shooter;
//     m_setpoint = setpoint;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(m_Shooter);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_WristTimer.start();
//     m_WheelTimer.start();
//     m_Shooter.setWristPos(m_setpoint.getAsDouble());
//     m_Shooter.setFlywheelSpeed(m_.getAsDouble());
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if(!m_Shooter.isWristAtDesiredPosition()){
//       m_Timer.reset();
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return m_Timer.get() < ShooterConstants.targetTime;
//   }
// }
