package frc.robot.autostep;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class SolenoidStep extends AutoStep {

    DoubleSolenoid solenoid;
    Value state;

    public SolenoidStep(DoubleSolenoid solenoid, Value state) {
        super();
        this.solenoid = solenoid;
        this.state = state;
    }

    public void Begin() {
		solenoid.set(state);
    }

    public void Update() {
        isDone = true;
    }
}