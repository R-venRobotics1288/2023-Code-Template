// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  // private final XboxController m_controller = new XboxController(0);
  private final XboxController m_controller = new XboxController(1);
  private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1.5);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1.5);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  private boolean driving = true;
  private double speedMultiplier = 1.0; // For speed controll via button press

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(true);
    // m_swerve.updateOdometry();
  }

  @Override
  public void robotPeriodic() {
    m_swerve.robotPeriodic();
  }
  

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
    SmartDashboard.putNumber("Left Joystick X", m_controller.getLeftX());
    SmartDashboard.putNumber("Left Joystick Y", m_controller.getLeftY());
    SmartDashboard.putNumber("Right Joystick X", m_controller.getRawAxis(2));
    // m_swerve.teleopPeriodic();
    // if (m_controller.getRawButton(7)) {
    //   m_swerve.setWheelsToOffset();
    // }

  
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    if (m_controller.getRightBumper()) {
      speedMultiplier = .333;
    } else {
      speedMultiplier = 1.0;
    }

    // Not tested yet - 2/21/23
    if (m_controller.getRawButton(9) && m_controller.getRawButton(10))  {
      m_swerve.m_gyro.setYaw(0);
    }

    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), DriveConstants.deadBand))
            * Drivetrain.kMaxSpeed * speedMultiplier;

    // System.out.println("Begin iteration");
    // System.out.println("xSpeed: "+xSpeed);

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), DriveConstants.deadBand))
            * Drivetrain.kMaxSpeed * speedMultiplier;

    // System.out.println("ySpeed: "+ySpeed);

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRawAxis(2), DriveConstants.deadBand))
            * Drivetrain.kMaxAngularSpeed;

    if (driving && (Math.abs(m_controller.getLeftX()) > DriveConstants.deadBand || Math.abs(m_controller.getLeftY()) > DriveConstants.deadBand || Math.abs(m_controller.getRawAxis(2)) > DriveConstants.deadBand)) {
      m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
    } else {
      m_swerve.stop();
    }
   
  }
}
