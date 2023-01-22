package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
  private int m_moduleNumber;
  private final Rotation2d m_angleOffset;
  private Rotation2d m_previousAngle;

  private final TalonFX m_angleMotor;
  private final TalonFX m_driveMotor;
  private final CANCoder m_angleEncoder;

  private final SimpleMotorFeedforward m_feedForwardController =
      new SimpleMotorFeedforward(
          Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  public SwerveModule(final int moduleNumber, final SwerveModuleConstants moduleConstants) {
    this.m_moduleNumber = moduleNumber;
    this.m_angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    m_angleEncoder = new CANCoder(moduleConstants.encoder.id, moduleConstants.encoder.bus);
    configAngleEncoder();

    /* Angle Motor Config */
    m_angleMotor = new TalonFX(moduleConstants.angleMotor.id, moduleConstants.angleMotor.bus);
    configAngleMotor();

    /* Drive Motor Config */
    m_driveMotor = new TalonFX(moduleConstants.driveMotor.id, moduleConstants.driveMotor.bus);
    configDriveMotor();

    m_previousAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, final boolean isOpenLoop) {
    /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
    desiredState = CTREModuleState.optimize(desiredState, getState().angle);
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(m_angleEncoder.getAbsolutePosition());
  }

  public void resetToAbsolute() {
    final double absolutePosition =
        Conversions.degreesToFalcon(
            getCanCoder().getDegrees() - m_angleOffset.getDegrees(),
            Constants.Swerve.angleGearRatio);
    m_angleMotor.setSelectedSensorPosition(absolutePosition);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        Conversions.falconToMPS(
            m_driveMotor.getSelectedSensorVelocity(),
            Constants.Swerve.wheelCircumference,
            Constants.Swerve.driveGearRatio),
        getAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        Conversions.falconToMeters(
            m_driveMotor.getSelectedSensorPosition(),
            Constants.Swerve.wheelCircumference,
            Constants.Swerve.driveGearRatio),
        getAngle());
  }

  public int getModuleNumber() {
    return m_moduleNumber;
  }

  private void setSpeed(final SwerveModuleState desiredState, final boolean isOpenLoop) {
    if (isOpenLoop) {
      final double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
    } else {
      final double velocity =
          Conversions.MPSToFalcon(
              desiredState.speedMetersPerSecond,
              Constants.Swerve.wheelCircumference,
              Constants.Swerve.driveGearRatio);
      m_driveMotor.set(
          ControlMode.Velocity,
          velocity,
          DemandType.ArbitraryFeedForward,
          m_feedForwardController.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(final SwerveModuleState desiredState) {

    // Only change angle if the desired speed is greater than 1% of the maximum speed
    // This prevents jittering when driving at slow speeds
    final boolean speedIsSignificant =
        Math.abs(desiredState.speedMetersPerSecond) > 0.01 * Constants.Swerve.maxSpeed;

    final Rotation2d nextAngle;

    if (speedIsSignificant) {
      nextAngle = desiredState.angle;
    } else {
      nextAngle = m_previousAngle;
    }

    m_angleMotor.set(
        ControlMode.Position,
        Conversions.degreesToFalcon(nextAngle.getDegrees(), Constants.Swerve.angleGearRatio));

    m_previousAngle = nextAngle;
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(
        Conversions.falconToDegrees(
            m_angleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
  }

  private void configAngleEncoder() {
    m_angleEncoder.configFactoryDefault();
    m_angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  private void configAngleMotor() {
    m_angleMotor.configFactoryDefault();
    m_angleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
    m_angleMotor.setInverted(Constants.Swerve.angleMotorInvert);
    m_angleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
    resetToAbsolute();
  }

  private void configDriveMotor() {
    m_driveMotor.configFactoryDefault();
    m_driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
    m_driveMotor.setInverted(Constants.Swerve.driveMotorInvert);
    m_driveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
    m_driveMotor.setSelectedSensorPosition(0);
  }
}
