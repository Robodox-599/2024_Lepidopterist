// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDAnimation;
import frc.robot.Constants.LEDConstants.LEDColor;
import frc.robot.Constants.RobotConstants;

public class subsystem_LED extends SubsystemBase {
  /** Creates a new subsystem_LED. */
  private CANdle m_CANdle;

  private LEDColor m_LEDColor = LEDColor.None;

  private Color currentColor;

  private LEDAnimation m_Animation = null;
  private boolean m_HasAnimationStarted = false;
  private Timer m_AnimationTimer = new Timer();
  private double m_AnimationDurationSeconds = 0.0;

  public subsystem_LED() {
    m_CANdle = new CANdle(LEDConstants.CANdle_ID, RobotConstants.CANBus);
    m_CANdle.configLEDType(LEDStripType.GRB);
    m_CANdle.configBrightnessScalar(0.5);
    m_CANdle.configLOSBehavior(true); // TODO: May need changing if this breaks LEDS
    m_AnimationTimer.start();

    currentColor = new Color(255, 255, 255); // default

    if (RobotConstants.robotColor == Alliance.Blue) {
      currentColor = new Color(0, 0, 255);
    } else if (RobotConstants.robotColor == Alliance.Red) {
      currentColor = new Color(255, 0, 0);
    }

    SmartDashboard.putString("LED Color", currentColor.toHexString());
    m_CANdle.setLEDs((int) currentColor.red, (int) currentColor.blue, (int) currentColor.green);
  }

  public void setLEDColor(LEDColor color) {
    m_LEDColor = color;
  }

  public InstantCommand setLEDStateCommand(LEDColor color) {
    return new InstantCommand(() -> setLEDColor(color), this);
  }

  public LEDColor getLEDState() {
    return m_LEDColor;
  }

  public void toggleAmpCoop() {
    if (m_LEDColor == LEDColor.AmplifyLED) {
      m_LEDColor = LEDColor.CoopLED;
    } else {
      m_LEDColor = LEDColor.AmplifyLED;
    }
  }

  public InstantCommand toggleAmpCoopCommand() {
    return new InstantCommand(() -> toggleAmpCoop(), this);
  }

  @Override
  public void periodic() {
    switch (m_LEDColor) {
      case AmplifyLED:
        if (RobotConstants.robotColor == Alliance.Red) {
          currentColor = new Color(255, 0, 0);
        } else {
          currentColor = new Color(0, 0, 255);
        }
        break;

      case CoopLED:
        currentColor = new Color(255, 191, 0);
        break;

      case Shooting:
        currentColor = new Color(255, 0, 255);

        break;

      case Amping:
        currentColor = new Color(0, 191, 255);

        break;

      case Intook:
        currentColor = new Color(0, 255, 0);

        break;
      default:
        if (RobotConstants.robotColor == Alliance.Red) {
          currentColor = new Color(255, 0, 0);
        } else {
          currentColor = new Color(0, 0, 255);
        }
        break;
    }

    SmartDashboard.putString("LED Color", currentColor.toHexString());
    m_CANdle.setLEDs((int) currentColor.red, (int) currentColor.blue, (int) currentColor.green);
  }
}
