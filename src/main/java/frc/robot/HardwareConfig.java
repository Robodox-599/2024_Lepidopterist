package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import com.ctre.phoenix6.StatusSignal;

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

  public HardwareConfig() {
    // /* Intake Wrist Motor Config */
    // intakeWristMotorConfig = new TalonFXConfiguration();
    // intakeWristMotorConfig.Slot0.kP = IntakeConstants.WristMotorConstants.wristExtendKP;
    // intakeWristMotorConfig.Slot0.kI = IntakeConstants.WristMotorConstants.wristExtendKI;
    // intakeWristMotorConfig.Slot0.kD = IntakeConstants.WristMotorConstants.wristExtendKD;
    // intakeWristMotorConfig.Slot0.kS = IntakeConstants.WristMotorConstants.wristExtendKS;

    // intakeWristMotorConfig.Slot1.kP = IntakeConstants.WristMotorConstants.wristRetractKP;
    // intakeWristMotorConfig.Slot1.kI = IntakeConstants.WristMotorConstants.wristRetractKI;
    // intakeWristMotorConfig.Slot1.kD = IntakeConstants.WristMotorConstants.wristRetractKD;

    // // chat what in the fricking frickheck is he cooking
    // var motionMagicConfigs = intakeWristMotorConfig.MotionMagic;
    // motionMagicConfigs.MotionMagicCruiseVelocity =
    //     IntakeConstants.WristMotorConstants.maxWristVelocity;
    // motionMagicConfigs.MotionMagicAcceleration =
    // IntakeConstants.WristMotorConstants.maxWristAccel;

    /* Indexer Motor Config */
    indexerMotorConfig = new TalonFXConfiguration();
    indexerMotorConfig.Slot0.kP = IndexerConstants.indexerKP;
    indexerMotorConfig.Slot0.kI = IndexerConstants.indexerKI;
    indexerMotorConfig.Slot0.kD = IndexerConstants.indexerKD;
    indexerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    indexerMotorConfig.CurrentLimits.SupplyCurrentLimit = 30.0;

    /* Shooter Wrist Motor Configs */
    shooterWristConfig = new TalonFXConfiguration();
    shooterWristConfig.Slot0.kP = ShooterConstants.WristMotorConstants.wristKP;
    shooterWristConfig.Slot0.kI = ShooterConstants.WristMotorConstants.wristKI;
    shooterWristConfig.Slot0.kD = ShooterConstants.WristMotorConstants.wristKD;
    shooterWristConfig.Slot0.kV = ShooterConstants.WristMotorConstants.wristKV;
    shooterWristConfig.Slot0.kS = ShooterConstants.WristMotorConstants.wristKS;
    var shooterWristMM = shooterWristConfig.MotionMagic;
    shooterWristMM.MotionMagicCruiseVelocity =
        ShooterConstants.WristMotorConstants.maxWristVelocity;
    shooterWristMM.MotionMagicAcceleration = ShooterConstants.WristMotorConstants.maxWristAccel;
  }

  public TalonFXConfiguration getMotorConfig(int config) {
    TalonFXConfiguration[] configs = {
      intakeRollerConfig,
      intakeWristMotorConfig,
      indexerMotorConfig,
      leftFlywheelConfig,
      rightFlywheelConfig,
      shooterWristConfig
    };
    return configs[config];
  }
}
