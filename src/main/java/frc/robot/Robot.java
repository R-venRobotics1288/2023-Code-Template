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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  // private final XboxController d_controller = new XboxController(0);
  private final XboxController d_controller = new XboxController(1);
  private final XboxController o_controller = new XboxController(0);
  private final Drivetrain m_swerve = new Drivetrain();

  private final CraneArm m_crane = new CraneArm(o_controller);
  private final Claw m_claw = new Claw(o_controller);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1.5);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1.5);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  private boolean driving = true;
  private double speedMultiplier = 1.0; // For speed control via button press
  // private final Pneumatics m_pneumatics = new Pneumatics();

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(true);
    // m_swerve.updateOdometry();
  }

  @Override
  public void robotPeriodic() {
    // m_swerve.robotPeriodic();
    SmartDashboard.putNumber("Arm up/down Encoder", m_crane.encoderPosition());
    SmartDashboard.putNumber("Extension Encoder", m_crane.extenisonEncoder());

  }

  @Override
  public void teleopInit() {
    // m_pneumatics.setStartingState();
  }
  

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);

    displaySmartDashboard();
    m_crane.craneRun();
    m_claw.clawRun();
  }

  public void displaySmartDashboard() {
    SmartDashboard.putNumber("Left Joystick X", d_controller.getLeftX());
    SmartDashboard.putNumber("Left Joystick Y", d_controller.getLeftY());
    SmartDashboard.putNumber("Right Joystick X", d_controller.getRawAxis(2));
    m_swerve.swerveSmartDashboard();
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    if (d_controller.getRightBumper()) {
      speedMultiplier = .333;
    } else {
      speedMultiplier = 1.0;
    }

    // Not tested yet - 2/21/23
    if (d_controller.getRawButton(9) && d_controller.getRawButton(10))  {
      m_swerve.m_gyro.setYaw(0);
    }

    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(d_controller.getLeftY(), DriveConstants.deadBand))
            * Drivetrain.kMaxSpeed * speedMultiplier;

    // System.out.println("Begin iteration");
    // System.out.println("xSpeed: "+xSpeed);

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(d_controller.getLeftX(), DriveConstants.deadBand))
            * Drivetrain.kMaxSpeed * speedMultiplier;

    // System.out.println("ySpeed: "+ySpeed);

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(d_controller.getRawAxis(2), DriveConstants.deadBand))
            * Drivetrain.kMaxAngularSpeed;

    if (driving && (Math.abs(d_controller.getLeftX()) > DriveConstants.deadBand || Math.abs(d_controller.getLeftY()) > DriveConstants.deadBand || Math.abs(d_controller.getRawAxis(2)) > DriveConstants.deadBand)) {
      m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
    } else {
      m_swerve.stop();
    }
   
  }
}
