// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.subsystem_ShooterWheels;
import frc.robot.subsystems.subsystem_ShooterWrist;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.subsystem_LED;
import  edu.wpi.first.wpilibj.GenericHID;


public class isReady extends Command {
  /** Creates a new isReadyLEDs. */
  private final subsystem_ShooterWheels m_wheels;
  private final subsystem_ShooterWrist m_wrist;
  private final subsystem_LED m_LED;
  private final XboxController m_driverController; 
  private final CommandXboxController m_opController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public isReady(subsystem_ShooterWheels wheels, subsystem_ShooterWrist wrist, subsystem_LED led) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_wheels = wheels;
    m_wrist = wrist;
    m_LED = led;
    m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);
    
    addRequirements(m_LED);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (m_wheels.is_flywheel_at_desired_speed() && m_wrist.isWristAtDesiredPosition() && m_LED.getState()) {
    //   m_LED.setGreenLED();
    //   SmartDashboard.putBoolean("green",(m_wheels.is_flywheel_at_desired_speed() && m_wrist.isWristAtDesiredPosition() && m_LED.getState()));

    //   // return true;
    // } else {
    //   m_LED.setStandbyLED();
    //   SmartDashboard.putBoolean("green",(m_wheels.is_flywheel_at_desired_speed() && m_wrist.isWristAtDesiredPosition() && m_LED.getState()));

    //   // return false;
    // }

if (m_wheels.checkingBeam()){
m_driverController.setRumble(GenericHID.RumbleType.kBothRumble,0.5);

} else{
  m_driverController.setRumble(GenericHID.RumbleType.kBothRumble,0);
}

    if (m_wheels.is_flywheel_at_desired_speed() && m_wrist.isWristAtDesiredPosition()
    && m_LED.get_state_machine_method() == "shoot") {
      // m_LED.setGreenLED();
      // if (!(m_wheels.getFlywheelState() == Constants.ShooterFlywheelConstants.StowSpeed)){
        m_wheels.toggle_feeder(true);
        m_wheels.set_diverter_method(1);
      // }
    
      // SmartDashboard.putBoolean("green",(m_wheels.is_flywheel_at_desired_speed() && m_wrist.isWristAtDesiredPosition()));

      // return true;
    } else {
      // m_LED.setStandbyLED();
    if ( m_LED.get_state_machine_method() == "shoot" && !m_opController.a().getAsBoolean()){
      
        m_wheels.toggle_feeder(true);
        m_wheels.set_diverter_method(1);
      }
      else{
        m_wheels.toggle_feeder(false);
        if (m_wheels.get_diverter_state() ==2 ){
          m_wheels.set_diverter_method(2);
           } else{
        m_wheels.set_diverter_method(0);}
      }
      // SmartDashboard.putBoolean("green",(m_wheels.is_flywheel_at_desired_speed() && m_wrist.isWristAtDesiredPosition()));

      // return false;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
m_wheels.toggle_feeder(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  return false;

  }
}
