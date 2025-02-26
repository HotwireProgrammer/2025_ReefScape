package frc.robot.autostep;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.swerve.DriveSubsystem;

public class SwerveAutoDriveStep extends AutoStep {

    DriveSubsystem swerve;
    float xSpeed;
    float ySpeed;
    float Spin;
    Timer timer = new Timer();
    float time;
    float cf;
    boolean fieldRelative;

    public SwerveAutoDriveStep(DriveSubsystem swerve, float xSpeed, float ySpeed, float spin, float time, boolean fieldRelative) {
        super();
        cf = 1.00f; // 2.00f is baseline for robot 0!
        this.fieldRelative = fieldRelative;
        this.swerve = swerve;
        this.xSpeed = xSpeed * cf;
        this.ySpeed = ySpeed * cf;
        Spin = spin;
        this.time = time;
    }

    public void Begin() {
        timer.reset();
        timer.start();
    }

    public void Update() {
        swerve.drive(xSpeed, ySpeed, Spin, fieldRelative, false);
        if (timer.get() > time) {
            isDone = true;
            swerve.drive(0, 0, 0, false, false);
        }
    }
}