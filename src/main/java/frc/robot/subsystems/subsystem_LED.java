// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.LEDConstants.LEDAnimation;
import frc.robot.Constants.LEDConstants.LEDColor;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;


public class subsystem_LED extends SubsystemBase {
  /** Creates a new subsystem_LED. */
  private CANdle m_CANdle;
  private LEDColor m_LEDColor = LEDColor.None;

  private LEDAnimation m_Animation = null;
  private boolean m_HasAnimationStarted = false;
  private Timer m_AnimationTimer = new Timer();
  private double m_AnimationDurationSeconds = 0.0;

  public subsystem_LED() {
    m_CANdle = new CANdle(Constants.LEDConstants.CANdle_ID);
    m_CANdle.configLEDType(LEDStripType.GRB);
    m_CANdle.configBrightnessScalar(0.5);
    m_CANdle.configLOSBehavior(true);
    SmartDashboard.putString("LED Color", new Color(0, 0, 0).toHexString());
    m_AnimationTimer.start();
  }

  public void setLEDColor(LEDColor color){
    m_LEDColor = color;
  }

  public InstantCommand setLEDStateCommand(LEDColor color){
    return new InstantCommand(() -> setLEDColor(color), this);
  }

  public LEDColor getLEDState(){
    return m_LEDColor;
  }

  public void toggleAmpCoop(){
    if (m_LEDColor == LEDColor.AmplifyLED){
      m_LEDColor = LEDColor.CoopLED;
    } else {
      m_LEDColor = LEDColor.AmplifyLED;
    }
  }

  public InstantCommand toggleAmpCoopCommand(){
    return new InstantCommand(() -> toggleAmpCoop(), this);
  }

  public void setAnimation(LEDAnimation animationToSet, double seconds){
    m_Animation = animationToSet;
    m_AnimationDurationSeconds = seconds;
    m_HasAnimationStarted = false;
    m_AnimationTimer.reset();
  }

  public InstantCommand setAnimationCommand (LEDAnimation animationToSet, DoubleSupplier seconds){
    return new InstantCommand(() -> setAnimation(animationToSet, seconds.getAsDouble()), this);
  }

  @Override
  public void periodic() {
    Color currentColor = new Color(0,0, 0);
    switch(m_LEDColor){
      case AmplifyLED:
        if (RobotConstants.robotColor == Alliance.Red){
          currentColor = new Color(255, 0, 0);
        } else {
          currentColor = new Color(0, 0, 255);
          SmartDashboard.putString("LED Color", new Color(0, 0, 255).toHexString());
        }
        break;
      
      case CoopLED:
        currentColor = new Color(255, 191, 0);
        break;
      
      default:
        currentColor = new Color(0, 0, 0);
        break;
    }

    SmartDashboard.putString("LED Color", currentColor.toHexString());

    //if we are animating
    if (m_Animation != null){
      //if the animation has already been set, check timer and cancel if necessary
      if (m_HasAnimationStarted){
        if (m_AnimationTimer.get() > m_AnimationDurationSeconds){
          m_Animation = null;
        }
      } else { // else start animation
        m_HasAnimationStarted = true;
        switch(m_Animation){
          case Intook:
            m_CANdle.animate(new StrobeAnimation((int) currentColor.red, (int) currentColor.blue, (int) currentColor.green));
          case Shooting:
            m_CANdle.animate(new ColorFlowAnimation((int) currentColor.red, (int) currentColor.blue, (int) currentColor.green));
          case Amping:
            FireAnimation fireAnim = new FireAnimation();
            fireAnim.setSpeed(0.75); 
            fireAnim.setSparking(1); 
            fireAnim.setCooling(0.3); 
            m_CANdle.animate(fireAnim);
        }
      }
    } else { //if no animation, set LEDS manually
      m_CANdle.setLEDs((int) currentColor.red, (int) currentColor.blue, (int) currentColor.green);
    }
  }
}
