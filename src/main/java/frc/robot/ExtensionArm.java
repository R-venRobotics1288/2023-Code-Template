package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ArmConstants;


public class ExtensionArm {
    private CANSparkMax m_extendingMotor;
    private RelativeEncoder m_extendEncoder;
    private XboxController o_controller;
    private static double extensionDesiredPosition = 0.0;

    private PIDController m_ExtensionPIDController = new PIDController(ArmConstants.extensionP, 0 ,0);

    public ExtensionArm(XboxController o_controller) {
        this.o_controller = o_controller;
        m_extendingMotor = new CANSparkMax(6, MotorType.kBrushless); // TODO Change ID
        m_extendEncoder = m_extendingMotor.getEncoder();
    }


    public void extendRun() {
        // Left Bumper - 5 - Arm Retraction
        if (o_controller.getRawButton(5)) {
            if (m_extendEncoder.getPosition() != ArmConstants.retractionLimit) {
                m_extendingMotor.set(-1);
            } 
            else {
                m_extendingMotor.set(0);
            }
        }
        // Left Trigger - 7 - Arm Extenison
        if (o_controller.getRawButton(7)) {
            if (m_extendEncoder.getPosition() != ArmConstants.extensionLimit) {
                m_extendingMotor.set(1);
            }
            else {
                m_extendingMotor.set(0);
            }
        }
        SmartDashboard.putNumber("Extension Encoder", m_extendEncoder.getPosition());
        SmartDashboard.putNumber("Extension Desired Position", extensionDesiredPosition);


    }

    public void buttonExtension(String position) {
        if (position.equals("ground")) {
            extensionDesiredPosition = ArmConstants.extendGround;
        } 
        if (position.equals("middle")) {
            extensionDesiredPosition = ArmConstants.extendMiddle;
        }
        if (position.equals("human")) {
            extensionDesiredPosition = ArmConstants.extendHuman;
        }
        if (position.equals("high")) {
            extensionDesiredPosition = ArmConstants.extendHigh;
        }
        final double extensionOutput = m_ExtensionPIDController.calculate(m_extendEncoder.getPosition(), extensionDesiredPosition);
        m_extendingMotor.set(extensionOutput);


        
    }
}
