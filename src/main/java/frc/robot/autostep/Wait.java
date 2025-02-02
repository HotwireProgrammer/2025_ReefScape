package frc.robot.autostep;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.swerve.DriveSubsystem;


public class Wait extends AutoStep {

    public Timer driveTimer;
    public float length;

    public Wait(Float length) {
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