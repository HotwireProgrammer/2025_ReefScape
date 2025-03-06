package frc.robot.autostep;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.swerve.DriveSubsystem;
import frc.robot.Limelight;


public class LimelightTrack extends AutoStep {

    public Timer limeTimer;
    public DriveSubsystem driveTrain;
    public Limelight limelight;

    public LimelightTrack(DriveSubsystem driveTrain, Limelight limelight) {
        super();
        this.driveTrain = driveTrain;
        this.limelight = limelight;
    }

    public void Begin() {
        limeTimer = new Timer();
        limeTimer.reset();
        limeTimer.start();
    }

    public void Update() {
        limelight.PositionRotate(driveTrain);

        runShooter = true;


        //limelight.Position(driveTrain);
        if (limelight.OnTargetHorizontal()) {
            limelight.reset();
            isDone = true;
        }
    }
}