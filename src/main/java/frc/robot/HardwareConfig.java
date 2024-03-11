package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.ctre.phoenix6.StatusSignal;

import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.UnitConstants;

public class HardwareConfig {
    CANcoderConfiguration swerveCANCoderConfig;
    TalonFXConfiguration swerveAngleFXConfig;
    TalonFXConfiguration swerveDriveFXConfig;
    TalonFXConfiguration intakeWristMotorConfig;
    TalonFXConfiguration indexerMotorConfig;
    TalonFXConfiguration leftFlywheelConfig;
    TalonFXConfiguration rightFlywheelConfig;
    TalonFXConfiguration shooterWristConfig;

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
        
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = SwerveConstants.openLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SwerveConstants.closedLoopRamp;
        swerveDriveFXConfig.Voltage.PeakForwardVoltage = UnitConstants.kNominal;

        /* Angle Motor Config */
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveAngleFXConfig.Slot0.kP = SwerveConstants.angleKP;
        swerveAngleFXConfig.Slot0.kI = SwerveConstants.angleKI;
        swerveAngleFXConfig.Slot0.kD = SwerveConstants.angleKD;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.angleContinuousCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = SwerveConstants.anglePeakCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = SwerveConstants.anglePeakCurrentDuration;

        /* Intake Wrist Motor Config */
        intakeWristMotorConfig = new TalonFXConfiguration();
        intakeWristMotorConfig.Slot0.kP = IntakeConstants.wristExtendKP;
        intakeWristMotorConfig.Slot0.kI = IntakeConstants.wristExtendKI;
        intakeWristMotorConfig.Slot0.kD = IntakeConstants.wristExtendKD;
        
        intakeWristMotorConfig.Slot1.kP = IntakeConstants.wristRetractKP;
        intakeWristMotorConfig.Slot1.kI = IntakeConstants.wristRetractKI;
        intakeWristMotorConfig.Slot1.kD = IntakeConstants.wristRetractKD;

        /* Indexer Motor Config */
        indexerMotorConfig = new TalonFXConfiguration();
        indexerMotorConfig.Slot0.kP = IndexerConstants.indexerKP;
        indexerMotorConfig.Slot0.kI = IndexerConstants.indexerKI;
        indexerMotorConfig.Slot0.kD = IndexerConstants.indexerKD;

        /* Left Flywheel Motor Configs */
        leftFlywheelConfig = new TalonFXConfiguration();
        leftFlywheelConfig.Slot0.kP = ShooterConstants.leftFlywheelKP;
        leftFlywheelConfig.Slot0.kI = ShooterConstants.leftFlywheelKI;
        leftFlywheelConfig.Slot0.kD = ShooterConstants.leftFlywheelKD;
        leftFlywheelConfig.Slot0.kS = ShooterConstants.leftFlywheelKS;
        leftFlywheelConfig.Slot0.kV = ShooterConstants.leftFlywheelKV;
        leftFlywheelConfig.Slot0.kA = ShooterConstants.leftFlywheelKA;
        
        /* Right Flywheel Motor Configs */
        rightFlywheelConfig = new TalonFXConfiguration();
        rightFlywheelConfig.Slot0.kP = ShooterConstants.rightFlywheelKP;
        rightFlywheelConfig.Slot0.kI = ShooterConstants.rightFlywheelKI;
        rightFlywheelConfig.Slot0.kD = ShooterConstants.rightFlywheelKD;
        rightFlywheelConfig.Slot0.kS = ShooterConstants.rightFlywheelKS;
        rightFlywheelConfig.Slot0.kV = ShooterConstants.rightFlywheelKV;
        rightFlywheelConfig.Slot0.kA = ShooterConstants.rightFlywheelKA;

        /* Shooter Wrist Motor Configs */
        shooterWristConfig = new TalonFXConfiguration();
        shooterWristConfig.Slot0.kP = ShooterConstants.wristKP;
        shooterWristConfig.Slot0.kI = ShooterConstants.wristKI;
        shooterWristConfig.Slot0.kD = ShooterConstants.wristKD;
        shooterWristConfig.Slot0.kV = ShooterConstants.wristKV;
        shooterWristConfig.Slot0.kS = ShooterConstants.wristKS;
    }

    public TalonFXConfiguration getMotorConfig(int config){
        TalonFXConfiguration[] configs = {intakeWristMotorConfig, 
                                        indexerMotorConfig,
                                        leftFlywheelConfig,
                                        rightFlywheelConfig,
                                        shooterWristConfig};
        return configs[config];
    }
}
