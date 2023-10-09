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
        if (o_controller.getRawButton(8)) {
            // Outtake
            clawMotor.set(.6);
        } else if (o_controller.getRawButton(6)) {
            // Intake
            clawMotor.set(-.85);
        } else {
            clawMotor.set(0);
        }
    }

    public void autoClawRun(String direction) {
        if (direction.equalsIgnoreCase("out")) {
            clawMotor.set(.6);
        } else if (direction.equalsIgnoreCase("in")) {
            clawMotor.set(-.85); 
        } else {
            clawMotor.set(0);
        }
    }
}
