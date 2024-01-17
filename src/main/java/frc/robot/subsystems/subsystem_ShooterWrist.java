

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import java.util.ResourceBundle.Control;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;


import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;

public class subsystem_ShooterWrist extends SubsystemBase {
/** Creates a new subsystem_Shooter. */

private TalonFX m_wrist;
private TalonFXConfiguration m_wristConfig;
private Encoder m_wrist_encoder;



private double m_desired_wrist_angle;




public subsystem_ShooterWrist() {
m_wrist = new TalonFX(Constants.MotorConstants.wristID);
m_wristConfig = new TalonFXConfiguration();
m_wrist_encoder = new Encoder(Constants.MotorConstants.DIO_pin1, Constants.MotorConstants.DIO_pin2);
m_wristConfig.Slot0.kP = Constants.PIDConstants.wristkP;
m_wristConfig.Slot0.kI = Constants.PIDConstants.wristkI;
m_wristConfig.Slot0.kD = Constants.PIDConstants.wristkD;


m_wrist.getConfigurator().apply(m_wristConfig);
}


public void set_wrist(double setpoint){


m_desired_wrist_angle = setpoint;


}


@Override
public void periodic() {
// This method will be called once per scheduler run


//if (//encoder shit)
PositionVoltage m_request2 = new PositionVoltage(0).withSlot(0);
m_wrist.setControl(m_request2.withPosition(m_desired_wrist_angle));
//put the if all systems good, ready on , led on



}
}


