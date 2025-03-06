package frc.robot.autostep;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public class ArmEjectStep extends AutoStep {

    Timer timer = new Timer();

    public ArmEjectStep() {
        super();
    }
    
    public void Begin() {
        timer.reset(); timer.start();
    }

    public void Update() {
        Robot.armMotorBottom.set(-0.7);
        Robot.armMotorTop.set(-0.7);

        if (timer.get() >= 0.3) {
            Robot.armMotorBottom.set(0.0);
            Robot.armMotorTop.set(0.0);
            isDone = true;
        }
    }
}