package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.Compressor;

public class Pneumatics {
    private DoubleSolenoid solenoid;
    private Compressor compressor;
    public Pneumatics() {
        solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,0,7);
        compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    }

    public void activate() {
        solenoid.toggle();
    }

    public void setStartingState() {
        solenoid.set(kForward);
    }
}
