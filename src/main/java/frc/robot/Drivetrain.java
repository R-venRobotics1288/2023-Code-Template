// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

// import edu.wpi.first.wpilibj.AnalogGyro;
import com.ctre.phoenix.sensors.PigeonIMU;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(DriveConstants.robotLength / 2, DriveConstants.robotWidth / 2);
  private final Translation2d m_frontRightLocation = new Translation2d(DriveConstants.robotLength / 2, -DriveConstants.robotWidth / 2);
  private final Translation2d m_backLeftLocation = new Translation2d(-DriveConstants.robotLength / 2, DriveConstants.robotWidth / 2);
  private final Translation2d m_backRightLocation = new Translation2d(-DriveConstants.robotLength / 2, -DriveConstants.robotWidth / 2);

    //CHANGE ENCODER PORTS
  private final SwerveModule m_frontLeft = new SwerveModule(1, 10, 20, DriveConstants.startingPositions[0]);
  private final SwerveModule m_frontRight = new SwerveModule(8, 2, 21, DriveConstants.startingPositions[1]);
  private final SwerveModule m_backLeft = new SwerveModule(7, 6, 22, DriveConstants.startingPositions[2]);
  private final SwerveModule m_backRight = new SwerveModule(3, 9, 23, DriveConstants.startingPositions[3]);

  public final PigeonIMU m_gyro = new PigeonIMU(30);
  public Camera camera = new Camera("raven1288");

  private double getGyroValue() {
    return m_gyro.getYaw() * Math.PI / 180;
  }

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          new Rotation2d(getGyroValue()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  // public void resteOffsets() {
  //     m_frontLeft.setDesiredState(DriveConstants.startingPositions[0]);
  // }

  public Drivetrain() {
    m_gyro.setYaw(0);
  }

  public void updateOdometry() {
    m_odometry.update(new Rotation2d(getGyroValue()), new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    });
    // TODO change from null
    Optional<EstimatedRobotPose> result = camera.getEstimatedGlobalPose(null);
    if (result.isPresent()) {
      
    }
  }

  
  public void swerveSmartDashboard() {
    SmartDashboard.putNumber("Front Left Desired Angle", m_frontLeft.targetAngle);
    SmartDashboard.putNumber("Front Right Desired Angle", m_frontRight.targetAngle);
    SmartDashboard.putNumber("Back Left Desired Angle", m_backLeft.targetAngle);
    SmartDashboard.putNumber("Back Right Desired Angle", m_backRight.targetAngle);
    SmartDashboard.putNumber("Front Left Abs Encoder Postion", m_frontLeft.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Front Right Abs Encoder Postion", m_frontRight.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Back Left Abs Encoder Postion", m_backLeft.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Back Right Abs Encoder Postion", m_backRight.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Gyro Yaw Value", getGyroValue());

  }

  public void robotInit() {
    SmartDashboard.putNumber("Front Left Abs Encoder Postion On Start", m_frontLeft.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Front Left Encoder On Start", m_frontLeft.getTurningEncoder().getPosition());
    SmartDashboard.putNumber("Front Left Encoder Offset", m_frontLeft.absoluteEncoderOffset);
  }


  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    final SwerveModuleState[] swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, new Rotation2d(getGyroValue()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void stop() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  public void setWheelsToOffset() {
    m_frontLeft.setStateToOffset();
    m_frontRight.setStateToOffset();
    m_backLeft.setStateToOffset();
    m_backRight.setStateToOffset();
  }
}
