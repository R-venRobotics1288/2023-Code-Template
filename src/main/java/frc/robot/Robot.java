// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;


/**
 * This sample program shows how to control a motor using a joystick. In the operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and speed controller inputs also range from -1 to 1
 * making it easy to work together.
 */
public class Robot extends TimedRobot {
  private XboxController xbox;
  private Pneumatics m_Pneumatics;
  @Override
  public void robotInit() {
    xbox = new XboxController(0);
    m_Pneumatics = new Pneumatics();
  }

  @Override
  public void teleopInit() {
    m_Pneumatics.setStartingState();
  }

  @Override
  public void teleopPeriodic() {
    if (xbox.getXButtonPressed()) {
      m_Pneumatics.activate();
    }
  }
}
