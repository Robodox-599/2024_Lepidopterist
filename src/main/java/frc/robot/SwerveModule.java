package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.UnitConstants;

public class SwerveModule {
  private TalonFX m_DriveMotor;
  private TalonFX m_AngleMotor;
  private CANcoder m_CANCoder;
  private double m_LastAngle;
  private double m_AngleOffset;
  private SimpleMotorFeedforward m_Feedforward;
  private HardwareConfig m_Settings;

  public SwerveModule(double[] module) {
    m_DriveMotor = new TalonFX((int) module[0], SwerveConstants.CANBus);
    m_AngleMotor = new TalonFX((int) module[1], SwerveConstants.CANBus);
    m_CANCoder = new CANcoder((int) module[2], SwerveConstants.CANBus);
    m_AngleOffset = module[3];
    m_Feedforward =
        new SimpleMotorFeedforward(
            SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);

    m_Settings = new HardwareConfig();

    /* CANCoder Config */
    // new __Configuration() replaces configFactoryDefault()
    m_CANCoder.getConfigurator().apply(new CANcoderConfiguration());
    m_CANCoder.getConfigurator().apply(m_Settings.swerveCANCoderConfig);

    /* Angle Motor Config */
    m_AngleMotor.getConfigurator().apply(new TalonFXConfiguration());
    m_AngleMotor.getConfigurator().apply(m_Settings.swerveAngleFXConfig);
    m_AngleMotor.setPosition(0.0);

    m_AngleMotor.setInverted(SwerveConstants.angleMotorInvert);
    m_AngleMotor.setNeutralMode(NeutralModeValue.Coast);
    // m_AngleMotor.setControl(new PositionVoltage(degreesToFalcon(m_AngleOffset -
    // getCANCoder().getDegrees())));
    m_AngleMotor.setPosition(degreesToFalcon(getCANCoder().getDegrees() - m_AngleOffset));

    /* Drive Motor Config */
    m_DriveMotor.getConfigurator().apply(new TalonFXConfiguration());
    m_DriveMotor.getConfigurator().apply(m_Settings.swerveDriveFXConfig);
    m_DriveMotor.setInverted(SwerveConstants.driveMotorInvert);
    m_DriveMotor.setNeutralMode(NeutralModeValue.Brake);
    m_DriveMotor.setPosition(0.0);
    m_LastAngle = getState().angle.getDegrees();
  }

  public double getTurnCounts() {
    return m_AngleMotor.getPosition().getValueAsDouble() * 360.0;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Optimize the state to possibly reverse the wheel direction if it results in less rotation of
    // the wheel.
    desiredState = optimize(desiredState, getState().angle);

    // Convert the desired linear speed to encoder units per 100 ms.
    double velocitySetpoint = mpsToFalcon(desiredState.speedMetersPerSecond);

    // Set the drive motor to the velocity setpoint using closed-loop control.

    m_DriveMotor.setControl(
        new VelocityVoltage(desiredState.speedMetersPerSecond).withVelocity(velocitySetpoint));

    // Set the angle motor to reach the desired angle using closed-loop control.

    double minSpeed = SwerveConstants.maxSpeed * 4.0 * 0.01;
    double angle =
        Math.abs(desiredState.speedMetersPerSecond) <= minSpeed
            ? m_LastAngle
            : desiredState.angle.getDegrees();
    m_AngleMotor.setControl(new PositionVoltage(degreesToFalcon(angle)).withSlot(0));
    m_LastAngle = angle;
  }

  public void setDesiredAngle(Rotation2d angle) {
    SwerveModuleState tempState = new SwerveModuleState(0.0, angle);
    tempState = optimize(tempState, getState().angle);
    m_AngleMotor.setControl(
        new PositionVoltage(degreesToFalcon(tempState.angle.getDegrees())).withSlot(0));
    m_LastAngle = angle.getDegrees();
  }

  public double getLastAngle() {
    return m_LastAngle;
  }

  public SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    double modReferenceAngleDeg =
        MathUtil.angleModulus(currentAngle.getRadians()) * UnitConstants.RAD_TO_DEG;
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = desiredState.angle.getDegrees() - modReferenceAngleDeg;
    if (delta >= 270.0) {
      delta -= 360.0;
    }
    if (delta <= -270.0) {
      delta += 360.0;
    }
    if (Math.abs(delta) > 90.0) {
      targetSpeed *= -1.0;
      delta = delta > 0.0 ? delta - 180.0 : delta + 180.0;
    }
    double targetAngle = currentAngle.getDegrees() + delta;
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  public void resetToAbsolute() {
    m_AngleMotor.setPosition(degreesToFalcon(getCANCoder().getDegrees() - m_AngleOffset));
  }

  public Rotation2d getCANCoder() {
    return Rotation2d.fromDegrees(m_CANCoder.getAbsolutePosition().getValueAsDouble() * 360.0);
  }

  public SwerveModulePosition getPosition() {
    double distance = falconToMeters(m_DriveMotor.getPosition().getValueAsDouble());
    Rotation2d angle =
        Rotation2d.fromDegrees(falconToDegrees(m_AngleMotor.getPosition().getValueAsDouble()));
    return new SwerveModulePosition(distance, angle);
  }

  public SwerveModuleState getState() {
    double velocity = falconToMPS(m_DriveMotor.getVelocity().getValueAsDouble());
    Rotation2d angle =
        Rotation2d.fromDegrees(falconToDegrees(m_AngleMotor.getPosition().getValueAsDouble()));
    return new SwerveModuleState(velocity, angle);
  }

  public double falconToMeters(double revolutions) {
    // return (counts * SwerveConstants.wheelCircumference) / (2048.0 *
    // SwerveConstants.driveGearRatio);
    return (revolutions * SwerveConstants.wheelCircumference) / SwerveConstants.driveGearRatio;
  }

  public double falconToDegrees(double revolutions) {
    // return (counts * 360.0) / (2048.0 * SwerveConstants.angleGearRatio);
    return (revolutions * 360.0) / SwerveConstants.angleGearRatio;
  }

  public double degreesToFalcon(double degrees) {
    // return degrees / (360.0 / (2048.0 * SwerveConstants.angleGearRatio));
    return (degrees / 360.0) * SwerveConstants.angleGearRatio;
  }

  public double falconToRPM(double velocityCounts) {
    // double motorRPM = velocityCounts * (600.0 / 2048.0); // Multiplying by 600 converts ms to min
    double motorRPM = velocityCounts * 60.0;
    return motorRPM / SwerveConstants.driveGearRatio;
  }

  public double rpmToFalcon(double rpm) {
    double motorRPM = rpm * SwerveConstants.driveGearRatio;
    // return motorRPM * (2048.0 / 600.0);
    return motorRPM / 60.0;
    // https://www.chiefdelphi.com/t/shooting-in-rpm-using-talon-fx/380660/2?u=bread
  }

  public double falconToMPS(double velocityCounts) {
    double wheelRPM = falconToRPM(velocityCounts);
    return wheelRPM * SwerveConstants.wheelCircumference / 60.0;
  }

  public double mpsToFalcon(double mps) {
    double wheelRPM = (mps * 60.0) / SwerveConstants.wheelCircumference;
    double falconVelocityUnits = rpmToFalcon(wheelRPM);
    return falconVelocityUnits;
  }

  public double getValFalc() {
    return degreesToFalcon(getCANCoder().getDegrees());
  }

  public double getValDegWithOffset() {
    return getCANCoder().getDegrees() - m_AngleOffset;
  }

  public String getErrorCodeUnits() {
    return m_AngleMotor.getClosedLoopError().getUnits();
  }

  public double getErrorCodeVal() {
    return m_AngleMotor.getClosedLoopError().getValueAsDouble();
  }
}
