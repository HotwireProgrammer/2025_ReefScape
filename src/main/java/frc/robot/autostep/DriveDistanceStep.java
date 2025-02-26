package frc.robot.autostep;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.studica.frc.AHRS;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.MathE;
import frc.robot.swerve.Constants;
import frc.robot.swerve.DriveSubsystem;

public class DriveDistanceStep extends AutoStep {

    double[] first;
    double[] second;
    Timer timer;
    DriveSubsystem swerve;
    boolean x_completion;
    boolean y_completion;
    double y_distance; // no exist
    double x_distance;
    double error;
    double distanceTraveled; // meters per second


    double maxRPM = 0; // apply gear ratio here
    double driveSpeed = 0.5; 

    public int[] m_IDs = {
        Constants.DriveConstants.kFrontLeftDrivingCanId, 
        Constants.DriveConstants.kRearLeftDrivingCanId, 
        Constants.DriveConstants.kFrontRightDrivingCanId,
        Constants.DriveConstants.kRearRightDrivingCanId
    };
    double[] m_encoders = {0, 0, 0, 0};

    final double maxMotorOutput = 0.8;


    // Maybe add a field realtive boolean so it can be toggled

    public DriveDistanceStep(DriveSubsystem swerve, double x_distance/*, double y_distance*/) { // Distance is in meters!
        super();
        this.swerve = swerve;
        this.x_distance = x_distance;
        timer = new Timer();
        // this.y_distance = y_distance;
    }

    

    public void Begin() {
        timer.reset(); timer.start();
        error = x_distance;
    }

    public void Update() {

        if (timer.get() > 0.05) {
            timer.reset();
            double speed = 5.74;//(0.073 /* Circumference of Wheel in m*/) * (((maxRPM * driveSpeed) / 60));
            if ((maxRPM * driveSpeed) < MathE.TAU/2) {
                distanceTraveled = (speed) / 20;
                System.out.println("distanceTraveled m/s = " + distanceTraveled * 20);
            }
            error -= distanceTraveled;
            swerve.drive(driveSpeed, 0, 0, false, false);
        }
        if (error <= 0) {
            swerve.drive(0, 0, 0, false, false);
            isDone = true;
        }
    }
}
