package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.ctre.phoenix6.StatusSignal;

import frc.robot.Constants.SwerveConstants;

public class HardwareConfig {
    CANcoderConfiguration swerveCANCoderConfig;
    TalonFXConfiguration swerveAngleFXConfig;
    TalonFXConfiguration swerveDriveFXConfig;

    public HardwareConfig(){
        /* CANCoder Config */
        swerveCANCoderConfig = new CANcoderConfiguration();
        swerveCANCoderConfig.MagnetSensor.AbsoluteSensorRange = SwerveConstants.RANGE_VALUE;
        swerveCANCoderConfig.MagnetSensor.SensorDirection = SwerveConstants.DIRECTION_VALUE;
        // Per second is default

        /* Drive Motor Config */
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig.Slot0.kP = SwerveConstants.driveKP;
        swerveDriveFXConfig.Slot0.kI = SwerveConstants.driveKI;
        swerveDriveFXConfig.Slot0.kD = SwerveConstants.driveKD;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.driveContinuousCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = SwerveConstants.drivePeakCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = SwerveConstants.drivePeakCurrentDuration;
        // SensorInitializationStrategy.BootToZero
        
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = SwerveConstants.openLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SwerveConstants.closedLoopRamp;
        swerveDriveFXConfig.Voltage.PeakForwardVoltage = SwerveConstants.kNominal;

        /* Angle Motor Config */
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveAngleFXConfig.Slot0.kP = SwerveConstants.angleKP;
        swerveAngleFXConfig.Slot0.kI = SwerveConstants.angleKI;
        swerveAngleFXConfig.Slot0.kD = SwerveConstants.angleKD;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.angleContinuousCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = SwerveConstants.anglePeakCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = SwerveConstants.anglePeakCurrentDuration;
        // SensorInitializationStrategy.BootToZero
    }
}
