package frc.robot.autostep;

import edu.wpi.first.wpilibj.Timer;


public class WaitStep extends AutoStep {

    public Timer driveTimer;
    public float length;

    public WaitStep(Float length) {
        super();
        this.length = length;
        driveTimer = new Timer();
    }

    public void Begin() {
        driveTimer = new Timer();
        driveTimer.reset();
        driveTimer.start();
    }

    public void Update() {
        if (driveTimer.get() > length) {
            isDone = true;
        }
    }
}