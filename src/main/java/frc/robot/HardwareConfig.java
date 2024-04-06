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
    TalonFXConfiguration intakeRollerConfig;

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
        intakeWristMotorConfig.Slot0.kP = IntakeConstants.WristMotorConstants.wristExtendKP;
        intakeWristMotorConfig.Slot0.kI = IntakeConstants.WristMotorConstants.wristExtendKI;
        intakeWristMotorConfig.Slot0.kD = IntakeConstants.WristMotorConstants.wristExtendKD;
        intakeWristMotorConfig.Slot0.kS = IntakeConstants.WristMotorConstants.wristExtendKS;
        
        intakeWristMotorConfig.Slot1.kP = IntakeConstants.WristMotorConstants.wristRetractKP;
        intakeWristMotorConfig.Slot1.kI = IntakeConstants.WristMotorConstants.wristRetractKI;
        intakeWristMotorConfig.Slot1.kD = IntakeConstants.WristMotorConstants.wristRetractKD;

        //chat what in the fricking frickheck is he cooking
        var motionMagicConfigs = intakeWristMotorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = IntakeConstants.WristMotorConstants.maxWristVelocity;
        motionMagicConfigs.MotionMagicAcceleration = IntakeConstants.WristMotorConstants.maxWristAccel;

        /* Indexer Motor Config */
        indexerMotorConfig = new TalonFXConfiguration();
        indexerMotorConfig.Slot0.kP = IndexerConstants.indexerKP;
        indexerMotorConfig.Slot0.kI = IndexerConstants.indexerKI;
        indexerMotorConfig.Slot0.kD = IndexerConstants.indexerKD;
        indexerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        indexerMotorConfig.CurrentLimits.SupplyCurrentLimit = 30.0;

        /* Left Flywheel Motor Configs */
        leftFlywheelConfig = new TalonFXConfiguration();
        leftFlywheelConfig.Slot0.kP = ShooterConstants.LeftFlywheelMotorConstants.leftFlywheelKP;
        leftFlywheelConfig.Slot0.kI = ShooterConstants.LeftFlywheelMotorConstants.leftFlywheelKI;
        leftFlywheelConfig.Slot0.kD = ShooterConstants.LeftFlywheelMotorConstants.leftFlywheelKD;
        leftFlywheelConfig.Slot0.kS = ShooterConstants.LeftFlywheelMotorConstants.leftFlywheelKS;
        leftFlywheelConfig.Slot0.kV = ShooterConstants.LeftFlywheelMotorConstants.leftFlywheelKV;
        leftFlywheelConfig.Slot0.kA = ShooterConstants.LeftFlywheelMotorConstants.leftFlywheelKA;
        
        leftFlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.LeftFlywheelMotorConstants.EnableCurrentLimit;
        leftFlywheelConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.LeftFlywheelMotorConstants.SupplyCurrentLimit;
        
        /* Right Flywheel Motor Configs */
        rightFlywheelConfig = new TalonFXConfiguration();
        rightFlywheelConfig.Slot0.kP = ShooterConstants.RightFlywheelMotorConstants.rightFlywheelKP;
        rightFlywheelConfig.Slot0.kI = ShooterConstants.RightFlywheelMotorConstants.rightFlywheelKI;
        rightFlywheelConfig.Slot0.kD = ShooterConstants.RightFlywheelMotorConstants.rightFlywheelKD;
        rightFlywheelConfig.Slot0.kS = ShooterConstants.RightFlywheelMotorConstants.rightFlywheelKS;
        rightFlywheelConfig.Slot0.kV = ShooterConstants.RightFlywheelMotorConstants.rightFlywheelKV;
        rightFlywheelConfig.Slot0.kA = ShooterConstants.RightFlywheelMotorConstants.rightFlywheelKA;

        rightFlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.RightFlywheelMotorConstants.EnableCurrentLimit;
        rightFlywheelConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.RightFlywheelMotorConstants.SupplyCurrentLimit;

        /* Shooter Wrist Motor Configs */
        shooterWristConfig = new TalonFXConfiguration();
        shooterWristConfig.Slot0.kP = ShooterConstants.WristMotorConstants.wristKP;
        shooterWristConfig.Slot0.kI = ShooterConstants.WristMotorConstants.wristKI;
        shooterWristConfig.Slot0.kD = ShooterConstants.WristMotorConstants.wristKD;
        shooterWristConfig.Slot0.kV = ShooterConstants.WristMotorConstants.wristKV;
        shooterWristConfig.Slot0.kS = ShooterConstants.WristMotorConstants.wristKS;
        var shooterWristMM = shooterWristConfig.MotionMagic;
        shooterWristMM.MotionMagicCruiseVelocity = ShooterConstants.WristMotorConstants.maxWristVelocity;
        shooterWristMM.MotionMagicAcceleration = ShooterConstants.WristMotorConstants.maxWristAccel;


        /* Intake Roller Config */
        intakeRollerConfig = new TalonFXConfiguration();
        intakeRollerConfig.Slot0.kP = IntakeConstants.RollerMotorConstants.kP;
        intakeRollerConfig.Slot0.kI = IntakeConstants.RollerMotorConstants.kI;
        intakeRollerConfig.Slot0.kD = IntakeConstants.RollerMotorConstants.kD;
        intakeRollerConfig.Slot0.kV = IntakeConstants.RollerMotorConstants.kV;
        intakeRollerConfig.Slot0.kS = IntakeConstants.RollerMotorConstants.kS;
        intakeRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = IntakeConstants.RollerMotorConstants.EnableCurrentLimit;
        intakeRollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.RollerMotorConstants.ContinuousCurrentLimit;
        intakeRollerConfig.CurrentLimits.SupplyCurrentThreshold = IntakeConstants.RollerMotorConstants.PeakCurrentLimit;
        intakeRollerConfig.CurrentLimits.SupplyTimeThreshold = IntakeConstants.RollerMotorConstants.PeakCurrentDuration;
    }

    public TalonFXConfiguration getMotorConfig(int config){
        TalonFXConfiguration[] configs = {intakeRollerConfig, 
                                        intakeWristMotorConfig, 
                                        indexerMotorConfig,
                                        leftFlywheelConfig,
                                        rightFlywheelConfig,
                                        shooterWristConfig};
        return configs[config];
    }
}
