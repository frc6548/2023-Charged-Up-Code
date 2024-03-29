package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule {
  public int moduleNumber;

  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANCoder angleEncoder;

  /* Initialize PID drive and angle */
  private final SparkPIDController driveController;
  private final SparkPIDController angleController;

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANCoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor(moduleConstants);

    lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
    // Custom optimize command, since default WPILib optimize assumes
    // continuous controller which REV and CTRE are not

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  /**
   * 
   * @param desiredState
   * @param isOpenLoop
   */

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  /**
   * 
   * @param desiredState
   */
  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01))
        ? lastAngle
        : desiredState.angle;

    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
  }

  /**
   * 
   */
  public void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
    integratedAngleEncoder.setPosition(absolutePosition);
  }

  /**
   * 
   * @return SwerveModulePosition
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
  }

  /**
   * 
   */
  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
    angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  /**
   * 
   */
  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(Constants.SwerveConstants.angleContinuousCurrentLimit);
    angleMotor.setInverted(Constants.SwerveConstants.angleInvert);
    angleMotor.setIdleMode(Constants.SwerveConstants.angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(Constants.SwerveConstants.angleConversionFactor);
    angleController.setPositionPIDWrappingEnabled(true);
    angleController.setPositionPIDWrappingMinInput(-180.0);
    angleController.setPositionPIDWrappingMaxInput(180.0);
    angleController.setP(Constants.SwerveConstants.angleKP);
    angleController.setI(Constants.SwerveConstants.angleKI);
    angleController.setD(Constants.SwerveConstants.angleKD);
    angleController.setFF(Constants.SwerveConstants.angleKFF);
    angleMotor.enableVoltageCompensation(Constants.SwerveConstants.voltageComp);
    angleMotor.burnFlash();
    resetToAbsolute();
  }

  /**
   * 
   */
  private void configDriveMotor(SwerveModuleConstants modConstants) {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kVelocityOnly);
    driveMotor.setSmartCurrentLimit(Constants.SwerveConstants.driveContinuousCurrentLimit);
    driveMotor.setInverted(modConstants.driveInvert);
    driveMotor.setIdleMode(Constants.SwerveConstants.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(Constants.SwerveConstants.driveConversionVelocityFactor * modConstants.encoderScale);
    driveEncoder.setPositionConversionFactor(Constants.SwerveConstants.driveConversionPositionFactor * modConstants.encoderScale);
    driveController.setP(Constants.SwerveConstants.angleKP);
    driveController.setI(Constants.SwerveConstants.angleKI);
    driveController.setD(Constants.SwerveConstants.angleKD);
    driveController.setFF(Constants.SwerveConstants.angleKFF);
    driveMotor.enableVoltageCompensation(Constants.SwerveConstants.voltageComp);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  /**
   * 
   * @return Rotation2d
   */
  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  /**
   * 
   * @return SwerveModulateState
   */
  public SwerveModuleState getState() {
    double velocity = driveEncoder.getVelocity();
    return new SwerveModuleState(velocity, getAngle());
  }

}