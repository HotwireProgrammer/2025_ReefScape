package frc.robot.autostep;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.swerve.DriveSubsystem;

public class DanceAutoStep extends AutoStep {

    DriveSubsystem swerve;
    float xSpeed;
    float ySpeed;
    float Spin;
    Timer timer;
    float time;

    public DanceAutoStep(DriveSubsystem swerve, float time) {
        super();
        this.swerve = swerve;
        timer = new Timer();
        this.time = time;
    }

    public void Begin() {
        timer.reset();
        timer.start();
    }

    public void Update() {
        swerve.drive(xSpeed, ySpeed, 0.0f, false, true);
        if (timer.get() > time) {
            isDone = true;
            //
            swerve.drive(0, 0, 0, false, true);
            //
        }
    }
}