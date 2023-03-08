package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Claw {
    private XboxController o_controller;
    private CANSparkMax clawMotor;

    public Claw(XboxController o_controller) {
        this.o_controller = o_controller;
        clawMotor = new CANSparkMax(11, MotorType.kBrushless);
        clawMotor.setSmartCurrentLimit(80);
        clawMotor.burnFlash();
    }

    public void clawRun() {
        // * Intake - SUBJECT TO CHANGE
        if (o_controller.getRawButton(8)) {
            clawMotor.set(.3);
            // * Outtake - SUBJECT TO CHANGE
        } else if (o_controller.getRawButton(6)) {
            clawMotor.set(-.85); //TODO Fix claw speeds
        } else {
            clawMotor.set(0);
        }
    }
}
