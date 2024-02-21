

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
import edu.wpi.first.wpilibj.DigitalInput;


public class subsystem_ShooterWheels extends SubsystemBase {
/** Creates a new subsystem_Shooter. */
private TalonFX m_flywheel1;
private TalonFX m_flywheel2;
private TalonFXConfiguration m_flywheelConfig;
private TalonFX m_feeder;
private TalonFX m_diverter;
private int m_diverterState;

DigitalInput m_breaker;

//0 is off 1 is shooter 2 is shamper

private boolean m_feeder_go;


private double m_desired_flywheel_speed;
private boolean m_ready;


private boolean m_bb_check;

public subsystem_ShooterWheels() {
    m_flywheel1 = new TalonFX(Constants.ShooterFlywheelConstants.MotorConstants.flywheel1ID);
    m_flywheel2 = new TalonFX(Constants.ShooterFlywheelConstants.MotorConstants.flywheel2ID);
    m_feeder = new TalonFX(Constants.ShooterFlywheelConstants.MotorConstants.feederID);
    m_diverter = new TalonFX(Constants.ShooterFlywheelConstants.MotorConstants.diverterID);
    m_flywheelConfig = new TalonFXConfiguration();
    m_flywheelConfig.Slot0.kP = Constants.ShooterFlywheelConstants.PIDConstants.flywheelkP;
    m_flywheelConfig.Slot0.kI = Constants.ShooterFlywheelConstants.PIDConstants.flywheelkI;
    m_flywheelConfig.Slot0.kD = Constants.ShooterFlywheelConstants.PIDConstants.flywheelkD;
    m_flywheelConfig.Slot0.kS = Constants.ShooterFlywheelConstants.PIDConstants.flywheelkS;
    m_flywheelConfig.Slot0.kV = Constants.ShooterFlywheelConstants.PIDConstants.flywheelkV;


    m_breaker = new DigitalInput(Constants.ShooterFlywheelConstants.MotorConstants.breakerPort1);

    m_diverterState = 0;
    m_ready = false;
    
    m_bb_check = false;

    m_flywheel1.getConfigurator().apply(m_flywheelConfig);
    m_flywheel2.getConfigurator().apply(m_flywheelConfig);
    
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

// m_ready = false;

// m_flywheel1.getConfigurator().apply(m_flywheelConfig);
// m_flywheel2.getConfigurator().apply(m_flywheelConfig);
}


public void set_flywheels(double v_setpoint){

m_desired_flywheel_speed = v_setpoint;

}
public void set_diverter_method(int state){
m_diverterState = state;
}
public Command set_diverterCommand(int state){
    return this.runOnce(() -> set_diverter_method(state));
}

public int get_diverter_state(){
    return m_diverterState;
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
    return((Math.abs(m_flywheel2.getVelocity().getValueAsDouble() - 
    (m_desired_flywheel_speed-ShooterFlywheelConstants.differential)) 
    <= ShooterFlywheelConstants.FlywheelMargin) && 
    (Math.abs(m_flywheel1.getVelocity().getValueAsDouble() - m_desired_flywheel_speed)
     <= ShooterFlywheelConstants.FlywheelMargin));
}

public double getFlywheelState(){
    return m_desired_flywheel_speed;
}
// V = d * kS + d^. * kV + d^.. * kA

public boolean isBeamBroken(){
    return(!(m_breaker.get()));
  }

public boolean checkingBeam(){
    if (m_bb_check = true && isBeamBroken()){
        return true;
    }
    else{
        return false;
    }
}

@Override
public void periodic() {
//while diverter state = 2 then bb_check = true
//while diverter state = 0 then bb_check = false
//if bb_check = true and bb broken then broken = true. 
//in isReady check if broken = true and rumble controller. 

if (m_diverterState == 2){
    m_bb_check = true;
} else{
    m_bb_check = false;
}



//     SmartDashboard.putBoolean("wheels ok", is_flywheel_at_desired_speed());
// // This method will be called once per scheduler run
// SmartDashboard.putNumber("desired", m_desired_flywheel_speed);
// SmartDashboard.putNumber("flywheel1", m_flywheel1.getVelocity().getValueAsDouble());
// SmartDashboard.putNumber("flywheel2", m_flywheel2.getVelocity().getValueAsDouble());
if (checkingBeam()){
SmartDashboard.putBoolean("test in cond", isBeamBroken());
}
SmartDashboard.putBoolean("test nmw", isBeamBroken());

if (m_diverterState == 0){
    m_diverter.set(0);
} else if (m_diverterState == 1){
    m_diverter.set(Constants.ShooterFlywheelConstants.diverter_speed_shooter);
} else if (m_diverterState == 2){
    m_diverter.set(Constants.ShooterFlywheelConstants.diverter_speed_amper);
}
SmartDashboard.putNumber("dv state", m_diverterState);


if (m_feeder_go == true) {
m_feeder.setVoltage(ShooterFlywheelConstants.MotorConstants.feeder_voltage);
} else{
m_feeder.setVoltage(0);
}
// double voltageFeedforward1 =( m_Feedforward.calculate(m_desired_flywheel_speed)) / SwerveConstants.kNominal;

// double voltageFeedforward2 = (m_Feedforward.calculate(m_desired_flywheel_speed - ShooterFlywheelConstants.differential) )/ SwerveConstants.kNominal;

// m_flywheel1.setVoltage(voltageFeedforward1);
m_flywheel1.set(m_desired_flywheel_speed/100.0);
if (m_desired_flywheel_speed-ShooterFlywheelConstants.differential>0){
m_flywheel2.set((m_desired_flywheel_speed-ShooterFlywheelConstants.differential)/100.0);}

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

//put the if all systems good, ready on , led on (move to command)






}
}


