// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  // private final XboxController m_controller = new XboxController(0);
  private final Joystick m_controller = new Joystick(0);
  private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  private boolean driving = true;

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
    driveWithJoystick(false);
    SmartDashboard.putNumber("Joystick X", m_controller.getX());
    SmartDashboard.putNumber("Joystick Y", m_controller.getY());
    // m_swerve.teleopPeriodic();
  
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getY(), DriveConstants.deadBand))
            * Drivetrain.kMaxSpeed;

    System.out.println("Begin iteration");
    System.out.println("xSpeed: "+xSpeed);

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getX(), DriveConstants.deadBand))
            * Drivetrain.kMaxSpeed;

    System.out.println("ySpeed: "+ySpeed);

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getZ(), DriveConstants.deadBand))
            * Drivetrain.kMaxAngularSpeed;
    if (driving && (Math.abs(m_controller.getX()) > DriveConstants.deadBand || Math.abs(m_controller.getY()) > DriveConstants.deadBand)) {
      m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
    } else {
      m_swerve.stop();
    }
   
  }
}
