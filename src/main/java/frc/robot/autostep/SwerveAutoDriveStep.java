package frc.robot.autostep;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.swerve.DriveSubsystem;

public class SwerveAutoDriveStep extends AutoStep {

    DriveSubsystem swerve;
    float xSpeed;
    float ySpeed;
    float Spin;
    Timer timer;
    float time;

    public SwerveAutoDriveStep(DriveSubsystem swerve, float xSpeed, float ySpeed, float spin, float time) {
        super();
        this.swerve = swerve;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        Spin = spin;
        timer = new Timer();
        this.time = time;
    }

    public void Begin() {
        timer.reset();
        timer.start();
    }

    public void Update() {
        swerve.drive(xSpeed, ySpeed, Spin, true, true);
        if (timer.get() > time) {
            isDone = true;
            swerve.drive(0, 0, 0, true, true);
        }
    }
}