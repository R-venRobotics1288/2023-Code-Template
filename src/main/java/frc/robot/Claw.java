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
    }

    public void clawRun() {
        // * Intake - SUBJECT TO CHANGE
        if (o_controller.getRawButton(8)) {
            System.out.println("Intake");
            clawMotor.set(.5);
            // * Outtake - SUBJECT TO CHANGE
        } else if (o_controller.getRawButton(6)) {
            System.out.println("Outtake");
            clawMotor.set(-.5);
        } else {
            System.out.println("Stop");
            clawMotor.set(0);
        }
    }
}
