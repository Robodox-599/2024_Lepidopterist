package frc.robot;

// import java.util.Arrays;
// import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
// import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.ctre.phoenix6.StatusSignal;

import frc.robot.Constants.SwerveConstants;

public class HardwareConfig {
    CANcoderConfiguration swerveCANCoderConfig;
    TalonFXConfiguration swerveAngleFXConfig;
    TalonFXConfiguration swerveDriveFXConfig;
    TalonFXConfiguration climbMotorConfig;
    TalonFXConfiguration intakeMotorConfig;
    TalonFXConfiguration wristMotorConfig;
    TalonFXConfiguration indexerMotorConfig;

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

        /* Climb Motor Config */
        climbMotorConfig = new TalonFXConfiguration();
        climbMotorConfig.Slot0.kP = ClimbConstants.extendP;
        climbMotorConfig.Slot0.kI = ClimbConstants.extendI;
        climbMotorConfig.Slot0.kD = ClimbConstants.extendD;

        climbMotorConfig.Slot1.kP = ClimbConstants.balanceP;
        climbMotorConfig.Slot1.kI = ClimbConstants.balanceI;
        climbMotorConfig.Slot1.kD = ClimbConstants.balanceD;

        /* Intake Motor Config */
        intakeMotorConfig = new TalonFXConfiguration();
        intakeMotorConfig.Slot0.kP = IntakeConstants.intakeKP;
        intakeMotorConfig.Slot0.kI = IntakeConstants.intakeKI;
        intakeMotorConfig.Slot0.kD = IntakeConstants.intakeKD;

        /* Wrist Motor Config */
        wristMotorConfig = new TalonFXConfiguration();
        wristMotorConfig.Slot0.kP = IntakeConstants.wristExtendKP;
        wristMotorConfig.Slot0.kI = IntakeConstants.wristExtendKI;
        wristMotorConfig.Slot0.kD = IntakeConstants.wristExtendKD;
        
        wristMotorConfig.Slot1.kP = IntakeConstants.wristRetractKP;
        wristMotorConfig.Slot1.kI = IntakeConstants.wristRetractKI;
        wristMotorConfig.Slot1.kD = IntakeConstants.wristRetractKD;

        /* Indexer Motor Config */
        indexerMotorConfig = new TalonFXConfiguration();
        indexerMotorConfig.Slot0.kP = IndexerConstants.indexerKP;
        indexerMotorConfig.Slot0.kI = IndexerConstants.indexerKI;
        indexerMotorConfig.Slot0.kD = IndexerConstants.indexerKD;
    }

    public TalonFXConfiguration getMotorConfig(int config){
        TalonFXConfiguration[] configs = {climbMotorConfig, 
                                        intakeMotorConfig, 
                                        wristMotorConfig, 
                                        indexerMotorConfig};
        return configs[config];
    }
}
