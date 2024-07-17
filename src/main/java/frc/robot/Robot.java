// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private PigeonIMU pigeonIMU;
  private Boolean autonomousInitRan = false;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source

    Logger.addDataReceiver(new WPILOGWriter());
    if (DriverStation.isFMSAttached()) {
      // Only turn on logs if we are attached to FMS, but disable NTtables.
      // Logger.addDataReceiver(new WPILOGWriter());
    } else {
      // Otherwise, enable NT tables.
      Logger.addDataReceiver(new NT4Publisher());
    }

    if (Constants.getMode() == Constants.Mode.REPLAY) {
      String replayLogPath = LogFileUtil.findReplayLog();

      Logger.setReplaySource(new WPILOGReader(replayLogPath));
    }

    if (Constants.getMode() == Constants.Mode.REPLAY) {
      setUseTiming(true);
    }

    Logger.disableDeterministicTimestamps();

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.disableDeterministicTimestamps()

    // Start AdvantageKit logger
    Logger.start();

    // Starts recording to data log
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    m_robotContainer = new RobotContainer();
    pigeonIMU = new PigeonIMU(12);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // checkDSUpdate();
  }

  @Override
  public void disabledPeriodic() {
    // checkDSUpdate();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousInitRan = true;
    pigeonIMU.setFusedHeading(pigeonIMU.getYaw());
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // Optional<Alliance> ally = DriverStation.getAlliance();
    // if (ally.get() == Alliance.Red) {
    //   isred = ally;
    // }
    // schedule the autonomous command (example)
    // checkDSUpdate();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    // checkDSUpdate();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // checkDSUpdate();
  }

  @Override
  public void teleopInit() {
    if (!autonomousInitRan) {
      System.out.println("Autonomous Init didn't before Teleop Init");
      pigeonIMU.setFusedHeading(pigeonIMU.getYaw());
      // Optional<Alliance> ally = DriverStation.getAlliance();
      // isred = ally;
    }
    // checkDSUpdate();
    // if (RobotConstants.robotColor == Alliance.Red){
    //   m_robotContainer.m_DriveTrain.SertGyro();
    // }

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // checkDSUpdate();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    // checkDSUpdate();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // checkDSUpdate();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    // checkDSUpdate();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    // checkDSUpdate();
  }
}
