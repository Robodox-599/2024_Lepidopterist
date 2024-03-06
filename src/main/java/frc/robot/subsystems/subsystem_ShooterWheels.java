

package frc.robot.subsystems;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterFlywheelConstants;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Volts;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;

import java.util.ResourceBundle.Control;
import java.util.function.BooleanSupplier;

import org.opencv.features2d.Feature2D;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
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
private TalonFX m_flywheel1;
private TalonFX m_flywheel2;
private TalonFXConfiguration m_flywheelConfig1;
private TalonFXConfiguration m_flywheelConfig2;
private TalonFX m_feeder;
private boolean sourcing;

private boolean m_feeder_go;


private double m_desired_flywheel_speed;
public boolean m_ready;
private SimpleMotorFeedforward feedforward;


public subsystem_ShooterWheels() {
    m_flywheel1 = new TalonFX(Constants.ShooterFlywheelConstants.MotorConstants.flywheel1ID, Constants.ShooterWristConstants.Canbus);
    m_flywheel2 = new TalonFX(Constants.ShooterFlywheelConstants.MotorConstants.flywheel2ID, Constants.ShooterWristConstants.Canbus);
    m_feeder = new TalonFX(Constants.ShooterFlywheelConstants.MotorConstants.feederID, Constants.ShooterWristConstants.Canbus);
    m_flywheelConfig1 = new TalonFXConfiguration();
    m_flywheelConfig2 = new TalonFXConfiguration();

    m_flywheelConfig1.Slot0.kP = Constants.ShooterFlywheelConstants.PIDConstants.flywheelkP1;
    m_flywheelConfig1.Slot0.kI = Constants.ShooterFlywheelConstants.PIDConstants.flywheelkI1;
    m_flywheelConfig1.Slot0.kD = Constants.ShooterFlywheelConstants.PIDConstants.flywheelkD1;
    m_flywheelConfig1.Slot0.kS = Constants.ShooterFlywheelConstants.PIDConstants.flywheelkS1;
    m_flywheelConfig1.Slot0.kV = Constants.ShooterFlywheelConstants.PIDConstants.flywheelkV1;
    m_flywheelConfig1.Slot0.kA = Constants.ShooterFlywheelConstants.PIDConstants.flywheelkA1;
    
    m_flywheelConfig2.Slot0.kP = Constants.ShooterFlywheelConstants.PIDConstants.flywheelkP2;
    m_flywheelConfig2.Slot0.kI = Constants.ShooterFlywheelConstants.PIDConstants.flywheelkI2;
    m_flywheelConfig2.Slot0.kD = Constants.ShooterFlywheelConstants.PIDConstants.flywheelkD2;
    m_flywheelConfig2.Slot0.kS = Constants.ShooterFlywheelConstants.PIDConstants.flywheelkS2;
    m_flywheelConfig2.Slot0.kV = Constants.ShooterFlywheelConstants.PIDConstants.flywheelkV2;
    m_flywheelConfig2.Slot0.kA = Constants.ShooterFlywheelConstants.PIDConstants.flywheelkA2;

    m_ready = false;
    sourcing = false;
    // feedforward = new SimpleMotorFeedforward(Constants.ShooterFlywheelConstants.PIDConstants.flywheelkS,
    // Constants.ShooterFlywheelConstants.PIDConstants.flywheelkV, 
    // Constants.ShooterFlywheelConstants.PIDConstants.flywheelkA);

    m_flywheel1.getConfigurator().apply(m_flywheelConfig1);
    m_flywheel2.getConfigurator().apply(m_flywheelConfig2);
    
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
public void source_feeder(boolean on){
sourcing = on;
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

@Override
public void periodic() {
    SmartDashboard.putBoolean("wheels ok", is_flywheel_at_desired_speed());
// This method will be called once per scheduler run
SmartDashboard.putNumber("desired", m_desired_flywheel_speed);
SmartDashboard.putNumber("flywheel1", m_flywheel1.getVelocity().getValueAsDouble());
SmartDashboard.putNumber("flywheel2", m_flywheel2.getVelocity().getValueAsDouble());


if (m_feeder_go == true) {
// m_feeder.setVoltage(ShooterFlywheelConstants.MotorConstants.feeder_voltage);
m_feeder.set(-0.5);
} else if (sourcing == true){
    // m_feeder.setVoltage(-ShooterFlywheelConstants.MotorConstants.feeder_voltage);
    m_feeder.set(0.5);

} else{
m_feeder.setVoltage(0);
m_feeder.set(0.0);


}




final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
m_flywheel1.setControl(m_request.withVelocity(m_desired_flywheel_speed).withFeedForward(0));

if (m_desired_flywheel_speed-ShooterFlywheelConstants.differential>0){
final VelocityVoltage m_request2 = new VelocityVoltage(0).withSlot(0);
m_flywheel2.setControl(m_request2.withVelocity(m_desired_flywheel_speed-Constants.ShooterFlywheelConstants.differential).withFeedForward(0));
}






// double voltageFeedforward1 =( m_Feedforward.calculate(m_desired_flywheel_speed)) / SwerveConstants.kNominal;

// double voltageFeedforward2 = (m_Feedforward.calculate(m_desired_flywheel_speed - ShooterFlywheelConstants.differential) )/ SwerveConstants.kNominal;

// m_flywheel1.setVoltage(voltageFeedforward1);


// m_flywheel1.setVoltage(pid.calculate(m_flywheel1.getVelocity(),m_desired_flywheel_speed)+feedforward.calculate(m_desired_flywheel_speed));


// m_flywheel1.set(ControlMode.Velocity, m_desired_flywheel_speed, DemandType.ArbitraryFeedForward, calculate(m_desired_flywheel_speed));
// if (m_desired_flywheel_speed-ShooterFlywheelConstants.differential>0){
// m_flywheel2.set(feedforward.calculate(m_desired_flywheel_speed-ShooterFlywheelConstants.differential));}

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














// m_flywheel1.set(m_desired_flywheel_speed);
// if (m_desired_flywheel_speed-ShooterFlywheelConstants.differential>0){
// m_flywheel2.set(m_desired_flywheel_speed-ShooterFlywheelConstants.differential);}














// use in case the other dont work
// if (((m_flywheel1.getVelocity().getValueAsDouble() - m_desired_flywheel_speed)/m_desired_flywheel_speed)>0.05){
//     m_flywheel1.set(feedforward.calculate(m_desired_flywheel_speed));
// }
// else{
//     final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
//     m_flywheel1.setControl(m_request.withVelocity(m_desired_flywheel_speed).withFeedForward(0));
// }
// // m_flywheel1.set(feedforward.calculate(m_desired_flywheel_speed));

// if (((m_flywheel2.getVelocity().getValueAsDouble() - (m_desired_flywheel_speed-ShooterFlywheelConstants.differential))
// /(m_desired_flywheel_speed-ShooterFlywheelConstants.differential))>0.05){
//     if (m_desired_flywheel_speed-ShooterFlywheelConstants.differential>0){
//     m_flywheel2.set(feedforward.calculate(m_desired_flywheel_speed-ShooterFlywheelConstants.differential));}
// }
// else{
//     if (m_desired_flywheel_speed-ShooterFlywheelConstants.differential>0){
//     final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
//     m_flywheel2.setControl(m_request.withVelocity(m_desired_flywheel_speed-ShooterFlywheelConstants.differential).withFeedForward(0));}
// }

}
}


