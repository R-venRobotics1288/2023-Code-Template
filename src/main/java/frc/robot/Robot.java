// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.CAN;


/**
 * This sample program shows how to control a motor using a joystick. In the operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and speed controller inputs also range from -1 to 1
 * making it easy to work together.
 */
public class Robot extends TimedRobot {
  private DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,1,2);

  @Override
  public void robotInit() {
    
  }

  @Override
  public void teleopPeriodic() {
    
  }
}
