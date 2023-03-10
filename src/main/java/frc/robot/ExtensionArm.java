package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ArmConstants;


public class ExtensionArm {
    public CANSparkMax m_extendingMotor;
    private RelativeEncoder m_extendEncoder;
    private XboxController o_controller;
    public double extensionDesiredPosition;
    public double extensionOutput;

    private PIDController m_ExtensionPIDController = new PIDController(ArmConstants.extensionP, 0 ,0);

    public ExtensionArm(XboxController o_controller) {
        this.o_controller = o_controller;
        m_extendingMotor = new CANSparkMax(5, MotorType.kBrushless);
        m_extendingMotor.setIdleMode(IdleMode.kCoast);
        m_extendingMotor.burnFlash();
        m_extendEncoder = m_extendingMotor.getEncoder();
        extensionDesiredPosition = 0;
        extensionOutput = 0;
    }


    public void extendRun() {
        // Neagtive extension, Postive retraction
        
        // // Left Bumper - 5 - Arm Retraction
        // if (o_controller.getRawButton(5)) {
        //     if (m_extendEncoder.getPosition() <= ArmConstants.retractionLimit) {
        //         m_extendingMotor.set(-5);
        //     } 
        //     else {
        //         m_extendingMotor.set(0);
        //     }
        // }
        // // Left Trigger - 7 - Arm Extenison
        // if (o_controller.getRawButton(7)) {
        //     if (m_extendEncoder.getPosition() <= ArmConstants.extensionLimit) {
        //         m_extendingMotor.set(5);
        //     }
        //     else {
        //         m_extendingMotor.set(0);
        //     }
        // }


        // SmartDashboard.putNumber("Extension Encoder", m_extendEncoder.getPosition());
        // SmartDashboard.putNumber("Extension Desired Position", extensionDesiredPosition);


    }

    public double extenisonEncoder() {
        return m_extendEncoder.getPosition();
    }

    public void buttonExtension(String position, boolean manual) {
        // Neagtive extension, Postive retraction
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
        if (position.equals("drive")) {
            extensionDesiredPosition = ArmConstants.retractionLimit;
        }
        if (extensionDesiredPosition < ArmConstants.retractionLimit) {
            extensionDesiredPosition = ArmConstants.retractionLimit;
        }
        if (extensionDesiredPosition > ArmConstants.extensionLimit) {
            extensionDesiredPosition = ArmConstants.extensionLimit;
        }
        final double extensionOutput = m_ExtensionPIDController.calculate(m_extendEncoder.getPosition(), extensionDesiredPosition);
        if (!manual) {
            m_extendingMotor.set(extensionOutput);
        }
        
    }

    public double getExtensionOutput() {
        return extensionOutput;
    }

}
