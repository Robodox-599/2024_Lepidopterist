

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import java.util.ResourceBundle.Control;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;


import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;

public class subsystem_ShooterWheels extends SubsystemBase {
/** Creates a new subsystem_Shooter. */
private TalonFX m_flywheel1;
private TalonFX m_flywheel2;
private TalonFXConfiguration m_flywheelConfig;
private TalonFX m_feeder;


private boolean m_feeder_go;


private double m_desired_flywheel_speed;
public boolean m_ready;




public subsystem_ShooterWheels() {
m_flywheel1 = new TalonFX(Constants.ShooterFlywheelConstants.MotorConstants.flywheel1ID);
m_flywheel2 = new TalonFX(Constants.ShooterFlywheelConstants.MotorConstants.flywheel2ID);
m_feeder = new TalonFX(Constants.ShooterFlywheelConstants.MotorConstants.feederID);
m_flywheelConfig = new TalonFXConfiguration();
m_flywheelConfig.Slot0.kP = Constants.ShooterFlywheelConstants.PIDConstants.flywheelkP;
m_flywheelConfig.Slot0.kI = Constants.ShooterFlywheelConstants.PIDConstants.flywheelkI;
m_flywheelConfig.Slot0.kD = Constants.ShooterFlywheelConstants.PIDConstants.flywheelkD;
m_flywheelConfig.Slot0.kS = Constants.ShooterFlywheelConstants.PIDConstants.flywheelkS;
m_flywheelConfig.Slot0.kV = Constants.ShooterFlywheelConstants.PIDConstants.flywheelkV;

m_ready = false;

m_flywheel1.getConfigurator().apply(m_flywheelConfig);
m_flywheel2.getConfigurator().apply(m_flywheelConfig);
}


public void set_flywheels(double v_setpoint){


m_desired_flywheel_speed = v_setpoint;


}
public void toggle_feeder(boolean shoot){

if (!((m_feeder_go == true && shoot == true) || (m_feeder_go == false && shoot ==false))){
m_feeder_go = shoot;

}


}
public Command toggle_feeder_Command(BooleanSupplier shoot){
    return this.runOnce(() -> toggle_feeder(shoot.getAsBoolean()));
}
public boolean is_flywheel_at_desired_speed(){
    return((Math.abs(m_flywheel2.getVelocity().getValueAsDouble() - m_desired_flywheel_speed) <= Constants.ShooterFlywheelConstants.FlywheelMargin) && 
    (Math.abs(m_flywheel1.getVelocity().getValueAsDouble() - m_desired_flywheel_speed) <= Constants.ShooterFlywheelConstants.FlywheelMargin));
}


@Override
public void periodic() {
// This method will be called once per scheduler run


if (m_feeder_go = true) {
m_feeder.setVoltage(Constants.ShooterFlywheelConstants.MotorConstants.feeder_voltage);
} else{
m_feeder.setVoltage(0);
}

if ((Math.abs(m_flywheel1.getVelocity().getValueAsDouble() - m_desired_flywheel_speed) > Constants.ShooterFlywheelConstants.FlywheelMargin)){
VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
m_flywheel1.setControl(m_request.withVelocity(m_desired_flywheel_speed).withFeedForward(0.5));
}


if ((Math.abs(m_flywheel2.getVelocity().getValueAsDouble() - m_desired_flywheel_speed) > Constants.ShooterFlywheelConstants.FlywheelMargin)){
VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
m_flywheel2.setControl(m_request.withVelocity(m_desired_flywheel_speed).withFeedForward(0.5));
}

//if (//encoder shit) then dont go 
//put the if all systems good, ready on , led on (move to command)






}
}


