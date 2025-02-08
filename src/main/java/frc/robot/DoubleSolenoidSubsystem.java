package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class DoubleSolenoidSubsystem {
    
    int PHreverseChannel; // This will be the port on your PH for reverse solenoid movement.
    int PHforwardChannel; // This will be the port on your PH for forward solenoid movement.
    
    DoubleSolenoid solenoid;
    boolean solenoidState = false;
    int PHid = 50; // This will be your Pneumatic Hub ID

    public DoubleSolenoidSubsystem(int PHforwardChannel, int PHreverseChannel, int PHid) {
        this.PHreverseChannel = PHreverseChannel;
        this.PHforwardChannel = PHforwardChannel;
        this.PHid = PHid; // On the 2025 robot this is/was 50

        solenoid = new DoubleSolenoid(PHid, PneumaticsModuleType.REVPH, PHforwardChannel, PHreverseChannel);
    }

    public void toggle() { // Toggles state of the solenoid.
        solenoidState = !solenoidState; update();
    }

    public void forward() { // Push Air
        solenoidState = true; update();}
    public void reverse() { // Pull Air
        solenoidState = true; update();}

    private void update() {
        if (solenoidState) {solenoid.set(Value.kForward);} else {solenoid.set(Value.kReverse);}}
}