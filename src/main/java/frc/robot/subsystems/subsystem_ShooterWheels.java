

package frc.robot.subsystems;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterFlywheelConstants;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;

import java.util.ResourceBundle.Control;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Command;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;

public class subsystem_ShooterWheels extends SubsystemBase {
/** Creates a new subsystem_Shooter. */
private CANSparkMax m_flywheel1;
private CANSparkMax m_flywheel2;
// private TalonFXConfiguration m_flywheelConfig;
private CANSparkMax m_feeder;

private SimpleMotorFeedforward m_Feedforward;

private SparkPIDController m_flywheel1PID;
private SparkPIDController m_flywheel2PID;
// private SparkPIDController m_feederPID;

private final RelativeEncoder m_flywheel1Encoder;
private final RelativeEncoder m_flywheel2Encoder;
private final RelativeEncoder m_feederEncoder;

private boolean m_feeder_go;


private double m_desired_flywheel_speed;
public boolean m_ready;




public subsystem_ShooterWheels() {
m_flywheel1 = new CANSparkMax(ShooterFlywheelConstants.MotorConstants.flywheel1ID,CANSparkLowLevel.MotorType.kBrushless);
m_flywheel2 = new CANSparkMax(ShooterFlywheelConstants.MotorConstants.flywheel2ID,CANSparkLowLevel.MotorType.kBrushless);
m_feeder = new CANSparkMax(ShooterFlywheelConstants.MotorConstants.feederID,CANSparkLowLevel.MotorType.kBrushless);

m_flywheel1Encoder = m_flywheel1.getEncoder();
m_flywheel2Encoder = m_flywheel2.getEncoder();
m_feederEncoder = m_feeder.getEncoder();

m_flywheel1Encoder.setPosition(0);
m_flywheel2Encoder.setPosition(0);

m_Feedforward = new SimpleMotorFeedforward(ShooterFlywheelConstants.PIDConstants.flywheelkS,
ShooterFlywheelConstants.PIDConstants.flywheelkV,
ShooterFlywheelConstants.PIDConstants.flywheelkA);

// m_flywheel1PID = m_flywheel1.getPIDController();
// m_flywheel2PID = m_flywheel2.getPIDController();
// m_feederPID = m_feeder.getPIDController();
// m_wrist = new CANSparkMax(Constants.ShooterWristConstants.MotorConstants.wristID,CANSparkLowLevel.MotorType.kBrushless);
// m_flywheel1PID.setP(ShooterFlywheelConstants.PIDConstants.flywheelkP);
// m_flywheel1PID.setI(ShooterFlywheelConstants.PIDConstants.flywheelkI);
// m_flywheel1PID.setD(ShooterFlywheelConstants.PIDConstants.flywheelkD);

// m_flywheel2PID.setP(ShooterFlywheelConstants.PIDConstants.flywheelkP);
// m_flywheel2PID.setI(ShooterFlywheelConstants.PIDConstants.flywheelkI);
// m_flywheel2PID.setD(ShooterFlywheelConstants.PIDConstants.flywheelkD); 

m_desired_flywheel_speed = 0;
m_feeder_go = false;

// // m_flywheelConfig = new TalonFXConfiguration();
// m_flywheelConfig.Slot0.kP = ShooterFlywheelConstants.PIDConstants.flywheelkP;
// m_flywheelConfig.Slot0.kI = ShooterFlywheelConstants.PIDConstants.flywheelkI;
// m_flywheelConfig.Slot0.kD = ShooterFlywheelConstants.PIDConstants.flywheelkD;
// m_flywheelConfig.Slot0.kS = ShooterFlywheelConstants.PIDConstants.flywheelkS;
// m_flywheelConfig.Slot0.kV = ShooterFlywheelConstants.PIDConstants.flywheelkV;

m_ready = false;

// m_flywheel1.getConfigurator().apply(m_flywheelConfig);
// m_flywheel2.getConfigurator().apply(m_flywheelConfig);
}


public void set_flywheels(double v_setpoint){


m_desired_flywheel_speed = v_setpoint;


}
// public void switch(){
//     m_feeder_go = !m_feeder_go;
// }

public void toggle_feeder(boolean shoot){

// if (!((m_feeder_go == true && shoot == true) || (m_feeder_go == false && shoot ==false))){
m_feeder_go = shoot;



// Timer.delay(ShooterFlywheelConstants.feeder_runtime);
// m_feeder_go = false;
}

// public void ()

// }
public Command toggle_feeder_Command(BooleanSupplier shoot){
    return this.runOnce(() -> toggle_feeder(shoot.getAsBoolean()));
}
public boolean is_flywheel_at_desired_speed(){
    return((Math.abs(m_flywheel2Encoder.getVelocity() - 
    (m_desired_flywheel_speed-ShooterFlywheelConstants.differential)) 
    <= ShooterFlywheelConstants.FlywheelMargin) && 
    (Math.abs(m_flywheel1Encoder.getVelocity() - m_desired_flywheel_speed)
     <= ShooterFlywheelConstants.FlywheelMargin));
}

public double getFlywheelState(){
    return m_desired_flywheel_speed;
}
// V = d * kS + d^. * kV + d^.. * kA

@Override
public void periodic() {
    SmartDashboard.putBoolean("wheels ok", is_flywheel_at_desired_speed());
// This method will be called once per scheduler run
SmartDashboard.putNumber("desired", m_desired_flywheel_speed);
SmartDashboard.putNumber("flywheel1", m_flywheel1Encoder.getVelocity());
SmartDashboard.putNumber("flywheel2", m_flywheel2Encoder.getVelocity());


if (m_feeder_go == true) {
m_feeder.setVoltage(ShooterFlywheelConstants.MotorConstants.feeder_voltage);
} else{
m_feeder.setVoltage(0);
}
double voltageFeedforward1 =( m_Feedforward.calculate(m_desired_flywheel_speed)) / SwerveConstants.kNominal;

double voltageFeedforward2 = (m_Feedforward.calculate(m_desired_flywheel_speed - ShooterFlywheelConstants.differential) )/ SwerveConstants.kNominal;

// m_flywheel1.setVoltage(voltageFeedforward1);
m_flywheel1.set(m_desired_flywheel_speed/5676.0);
if (m_desired_flywheel_speed-ShooterFlywheelConstants.differential>0){
m_flywheel2.set((m_desired_flywheel_speed-ShooterFlywheelConstants.differential)/5676.0);}

// if ((Math.abs(m_flywheel1Encoder.getVelocity() - m_desired_flywheel_speed)
//  > ShooterFlywheelConstants.FlywheelMargin)){
// // VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
// if (m_desired_flywheel_speed > 0){
// m_flywheel1PID.setReference(m_desired_flywheel_speed, CANSparkMax.ControlType.kVelocity, 0);
// }
// }
// m_flywheel1.setControl(m_request.withVelocity(m_desired_flywheel_speed).withFeedForward(0.5));
// } else{
//     // m_flywheel1.stopMotor();

// }

// if ((Math.abs(m_flywheel2Encoder.getVelocity() - (m_desired_flywheel_speed-ShooterFlywheelConstants.differential))
//  > ShooterFlywheelConstants.FlywheelMargin)){
//     // VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
//     if (m_desired_flywheel_speed-ShooterFlywheelConstants.differential>0){
//     m_flywheel2PID.setReference(m_desired_flywheel_speed-ShooterFlywheelConstants.differential,CANSparkMax.ControlType.kVelocity, 0);
// }}
    // m_flywheel2.setControl(m_request.withVelocity(m_desired_flywheel_speed).withFeedForward(0.5));
    // }else{
    //     // m_flywheel2.stopMotor();
    // }

//if (//encoder shit) then dont go 
//put the if all systems good, ready on , led on (move to command)






}
}


