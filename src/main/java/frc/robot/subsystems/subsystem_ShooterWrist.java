
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.Constants.ShooterWristConstants;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.revrobotics.CANSparkLowLevel;



public class subsystem_ShooterWrist extends SubsystemBase {
  /** Creates a new subsystem_Shooter. */

  // private TalonFX m_wrist;
  private TalonFX m_wrist;
  private TalonFXConfiguration m_wristConfig;
  private Encoder m_wristEncoder;

  private double m_desired_wrist_angle;  // private TalonFXConfiguration m_wristConfig;
  // private Encoder m_wristEncoder;
  // private AbsoluteEncoder absEncoder;
  private DutyCycleEncoder absEncoderDuty;
  // private SparkPIDController m_PidController;
// private final RelativeEncoder m_RelativeEncoder;
  // private double m_desired_wrist_angle;
  private double startPos;

  public subsystem_ShooterWrist() {
    m_wrist = new TalonFX(Constants.ShooterWristConstants.MotorConstants.wristID, Constants.ShooterWristConstants.Canbus);
    m_wristConfig = new TalonFXConfiguration();
    m_wristEncoder = new Encoder(Constants.ShooterWristConstants.MotorConstants.DIO_pin1, Constants.ShooterWristConstants.MotorConstants.DIO_pin2);
    m_wristConfig.Slot0.kP = Constants.ShooterWristConstants.PIDConstants.wristkP;
    m_wristConfig.Slot0.kI = Constants.ShooterWristConstants.PIDConstants.wristkI;
    m_wristConfig.Slot0.kD = Constants.ShooterWristConstants.PIDConstants.wristkD;

    m_wrist.getConfigurator().apply(m_wristConfig);
    // m_wristConfig = new TalonFXConfiguration();
    // m_RelativeEncoder = new Encoder(Constants.ShooterWristConstants.MotorConstants.DIO_pin1, Constants.ShooterWristConstants.MotorConstants.DIO_pin2);
    // m_wristConfig.Slot0.kP = Constants.ShooterWristConstants.PIDConstants.wristkP;
    // m_wristConfig.Slot0.kI = Constants.ShooterWristConstants.PIDConstants.wristkI;
    // m_wristConfig.Slot0.kD = Constants.ShooterWristConstants.PIDConstants.wristkD;
    // m_RelativeEncoder = m_wrist.getEncoder();
    // m_PidController = m_wrist.getPIDController();
    // absEncoderDuty = new DutyCycleEncoder(Constants.ShooterWristConstants.absEncoderChannel);
    // m_wrist.getConfigurator().apply(m_wristConfig);
    // absEncoderDuty.set
    startPos =m_wrist.getPosition().getValueAsDouble();
    
    m_desired_wrist_angle=0;
  }

  public void set_wrist(double angle) {
    if (angle >= ShooterWristConstants.minWrist && angle <= ShooterWristConstants.maxWrist)
    m_desired_wrist_angle = (angle / Constants.ShooterWristConstants.MotorConstants.rotations_to_angle) + startPos;
  }
  public Command test_wrist(double angle){
    return this.runOnce(()-> set_wrist(angle));
  }

public boolean isWristAtDesiredPosition(){
  // return Math.abs(m_wristEncoder.getRaw()*Constants.ShooterWristConstants.MotorConstants.ticks_to_rotations - m_desired_wrist_angle) 
  // <= ShooterWristConstants.wristError;
    return Math.abs((m_wrist.getPosition().getValueAsDouble()) - m_desired_wrist_angle) 
  <= ShooterWristConstants.wristError;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
SmartDashboard.putBoolean("wrist ok", isWristAtDesiredPosition());
    // if (//encoder shit)
    // if (!(Math.abs(m_wristEncoder.getRaw()*Constants.ShooterWristConstants.MotorConstants.ticks_to_rotations - m_desired_wrist_angle)>Constants.ShooterWristConstants.wristError)){
    // PositionVoltage m_request2 = new PositionVoltage(0).withSlot(0);
    // m_wrist.setControl(m_request2.withPosition(m_desired_wrist_angle));
    if (!isWristAtDesiredPosition()){
      PositionVoltage m_request2 = new PositionVoltage(0).withSlot(0);
      m_wrist.setControl(m_request2.withPosition(m_desired_wrist_angle));    // put the if all systems good, ready on , led on

    }else{
      m_wrist.stopMotor();
    }
    
  }
}
