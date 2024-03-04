
package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

  private double m_desired_wrist;  // private TalonFXConfiguration m_wristConfig;
  // private Encoder m_wristEncoder;
  // private AbsoluteEncoder absEncoder;
  private DutyCycleEncoder absEncoderDuty;
  // private SparkPIDController m_PidController;
// private final RelativeEncoder m_RelativeEncoder;
  // private double m_desired_wrist;
  private double startPos;
  private PIDController m_PidController;
  private double motor_speed;
// private TrapezoidProfile profile;
private TrapezoidProfile m_profile;

  public subsystem_ShooterWrist() {
    m_wrist = new TalonFX(Constants.ShooterWristConstants.MotorConstants.wristID, Constants.ShooterWristConstants.Canbus);
    m_wristConfig = new TalonFXConfiguration();
    m_wristEncoder = new Encoder(Constants.ShooterWristConstants.MotorConstants.DIO_pin1, Constants.ShooterWristConstants.MotorConstants.DIO_pin2);
    m_wristConfig.Slot0.kP = Constants.ShooterWristConstants.PIDConstants.wristkP;
    m_wristConfig.Slot0.kI = Constants.ShooterWristConstants.PIDConstants.wristkI;
    m_wristConfig.Slot0.kD = Constants.ShooterWristConstants.PIDConstants.wristkD;
    m_wristConfig.Slot0.kV = Constants.ShooterWristConstants.PIDConstants.wristkV;
    m_wristConfig.Slot0.kS = Constants.ShooterWristConstants.PIDConstants.wristkS;


    m_profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(Constants.ShooterWristConstants.MotorConstants.max_velocity,
       Constants.ShooterWristConstants.MotorConstants.max_accel)
   );

    m_PidController.setPID(Constants.ShooterWristConstants.PIDConstants.wristkP,
     Constants.ShooterWristConstants.PIDConstants.wristkI, 
     Constants.ShooterWristConstants.PIDConstants.wristkD);
    
    //  profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(Constants.ShooterWristConstants.MotorConstants.max_velocity,
    //  Constants.ShooterWristConstants.MotorConstants.max_accel));

    motor_speed =0;
    m_wrist.getConfigurator().apply(m_wristConfig);
    // m_wristConfig = new TalonFXConfiguration();
    // m_RelativeEncoder = new Encoder(Constants.ShooterWristConstants.MotorConstants.DIO_pin1, Constants.ShooterWristConstants.MotorConstants.DIO_pin2);
    // m_wristConfig.Slot0.kP = Constants.ShooterWristConstants.PIDConstants.wristkP;
    // m_wristConfig.Slot0.kI = Constants.ShooterWristConstants.PIDConstants.wristkI;
    // m_wristConfig.Slot0.kD = Constants.ShooterWristConstants.PIDConstants.wristkD;
    // m_RelativeEncoder = m_wrist.getEncoder();
    // m_PidController = m_wrist.getPIDController();
    absEncoderDuty = new DutyCycleEncoder(Constants.ShooterWristConstants.absEncoderChannel);
    // m_wrist.getConfigurator().apply(m_wristConfig);
    // absEncoderDuty.set
    startPos =absEncoderDuty.getAbsolutePosition();
    
    m_desired_wrist=0;
  }

  public void set_wrist(double angle) {
    // if (angle >= ShooterWristConstants.minWrist && angle <= ShooterWristConstants.maxWrist)
    m_desired_wrist = (angle / Constants.ShooterWristConstants.MotorConstants.rotations_to_angle) + startPos;
  }
  // public Command test_wrist(double angle){
  //   return this.runOnce(()-> set_wrist(angle));
  // }

public boolean isWristAtDesiredPosition(){
  // return Math.abs(m_wristEncoder.getRaw()*Constants.ShooterWristConstants.MotorConstants.ticks_to_rotations - m_desired_wrist) 
  // <= ShooterWristConstants.wristError;
    return Math.abs((absEncoderDuty.getAbsolutePosition() - m_desired_wrist) )
  <= ShooterWristConstants.wristError;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
SmartDashboard.putBoolean("wrist ok", isWristAtDesiredPosition());
SmartDashboard.putNumber("encoder",absEncoderDuty.getAbsolutePosition());
SmartDashboard.putNumber("desired", m_desired_wrist);
    // if (//encoder shit)
    // if (!(Math.abs(m_wristEncoder.getRaw()*Constants.ShooterWristConstants.MotorConstants.ticks_to_rotations - m_desired_wrist)>Constants.ShooterWristConstants.wristError)){
    // PositionVoltage m_request2 = new PositionVoltage(0).withSlot(0);
    // m_wrist.setControl(m_request2.withPosition(m_desired_wrist));
  

   //use if other fails
 
   // Final target of 200 rot, 0 rps
  //  TrapezoidProfile.State m_goal = new TrapezoidProfile.State(m_desired_wrist, 0);
  //  TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
   
  //  // create a position closed-loop request, voltage output, slot 0 configs
  //  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
   
  //  // calculate the next profile setpoint
  //  m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal);
   
  //  // send the request to the device
  //  m_request.Position = m_setpoint.position;
  //  m_request.Velocity = m_setpoint.velocity;
  //  m_wrist.setControl(m_request);
  
  
  
  
  
  
 
  
    if (!isWristAtDesiredPosition()){
      // PositionVoltage m_request2 = new PositionVoltage(0).withSlot(0);
      // m_wrist.setControl(m_request2.withPosition(m_desired_wrist));    // put the if all systems good, ready on , led on
      motor_speed = m_PidController.calculate(absEncoderDuty.getAbsolutePosition(), m_desired_wrist);
      m_wrist.set(motor_speed);
    }else{
      m_wrist.stopMotor();
    }
    
  }
}
