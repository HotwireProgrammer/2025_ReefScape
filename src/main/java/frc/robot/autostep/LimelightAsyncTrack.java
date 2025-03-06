package frc.robot.autostep;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Limelight;
import frc.robot.Robot;
import frc.robot.swerve.DriveSubsystem;

public class LimelightAsyncTrack extends AutoStep {

    public int ID;
    public double timeOut;
    public double speed;
    public DriveSubsystem swerveDrive;
    public Limelight limelight;
    public Timer timer = new Timer();

    public LimelightAsyncTrack(DriveSubsystem swerve, Limelight limelight, int ID, double speed) {
        super();
        this.swerveDrive = swerve;
        this.limelight = limelight;
        this.timeOut = 0;
        this.speed = speed;
        this.ID = ID;
    }
    public LimelightAsyncTrack(DriveSubsystem swerve, Limelight limelight, double speed) {
        super();
        this.swerveDrive = swerve;
        this.limelight = limelight;
        this.timeOut = 0;
        this.speed = speed;
        this.ID = -1;
    }
    public LimelightAsyncTrack(DriveSubsystem swerve, Limelight limelight, int ID, double speed, double timeOut) {
        super();
        this.swerveDrive = swerve;
        this.limelight = limelight;
        this.timeOut = timeOut;
        this.speed = speed;
        this.ID = ID;
    }
    
    public void Begin() {
        
        if (timeOut == 0) {timeOut = 10;}
        isDone = true;
        new Thread(() -> {
            timer.reset(); timer.start();
            System.out.println("Time Out = " + timeOut);

            while (true) {
                if (limelight.GetAprilID() == ID || limelight.GetAprilID() == -1) {
                    double turnDegree = limelight.GetX() + swerveDrive.m_gyro.getYaw();
                    swerveDrive.GoToRotation(turnDegree);
                }

                if (timer.get() >= timeOut) {
                    break;
                }
            }

            // swerveDrive.drive(speed, 0, 0, false, false);
        }).start();
    }

    public void Update() {
        
    }
}