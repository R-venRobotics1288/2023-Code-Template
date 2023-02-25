// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  public final CANCoder m_absoluteEncoder;
  public double absoluteEncoderOffset;

  public RelativeEncoder getTranslationEncoder() {
    return m_driveEncoder;
  }

  public RelativeEncoder getTurningEncoder() {
    return m_turningEncoder;
  }

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(0.25, 0, 0);

  private final PIDController m_turningPIDController = new PIDController(0.6, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.015, 0.285);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  // Testing variables for shuffleboard
  public double targetAngle;
  public double error;

  // public double getAbsoluteEncoder() {
  //   return m_absoluteEncoder.getAbsolutePosition();
  // }

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotorChannel      CAN id for the drive motor.
   * @param turningMotorChannel    CAN id for the turning motor.
   * @param turningEncoderChannel  CAN id for the absolute encoder
   * @param offset                 Offset for the motor. Basically the forward position value
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      double offset) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningMotor.setInverted(false);
    m_turningMotor.burnFlash();

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPosition(0);
    m_turningEncoder = m_turningMotor.getEncoder();
    m_absoluteEncoder = new CANCoder(turningEncoderChannel);

    // Configuration settings for Absolute Encoder
    CANCoderConfiguration config = new CANCoderConfiguration();
    config.sensorCoefficient = 2 * Math.PI / kEncoderResolution;
    config.unitString = "rad";
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    // config.magnetOffsetDegrees = offset * 180 / Math.PI;
    config.magnetOffsetDegrees = 0;
    config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;

    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    m_absoluteEncoder.configAllSettings(config);

    m_driveEncoder.setPositionConversionFactor(2 * Math.PI * kWheelRadius);
    m_turningEncoder.setPositionConversionFactor(12.8 * Math.PI * 2);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    absoluteEncoderOffset = offset;

    // resetEncoders();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_turningEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  // Gets the absolute encoder value in radians using the offset value
  public double getAbsoluteEncoderRad() {
    return m_absoluteEncoder.getAbsolutePosition();
  }

  // public void resetEncoders() {
  //   m_driveEncoder.setPosition(0);
  //   m_turningEncoder.setPosition(getAbsoluteEncoderRad() / DriveConstants.radiansPerEncoderRev);
  // }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    System.out.println("Before Optimize: " + getAbsoluteEncoderRad());
    desiredState.angle = desiredState.angle.plus(new Rotation2d(absoluteEncoderOffset));
    final SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getAbsoluteEncoderRad()));
    System.out.println("After Optimize: " + state);

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians());

    targetAngle = state.angle.getRadians();

    final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint());

    // m_driveMotor.setVoltage((driveOutput + driveFeedforward));
    m_driveMotor.set(driveOutput + driveFeedforward);
    m_turningMotor.set(turnOutput / 3);
  }

  public void setStateToOffset() {
    // Testing purposes only. Sets the desired state to the offset value
    setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
  }

  public void stop() {
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  }
}
