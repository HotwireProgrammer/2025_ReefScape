package frc.robot.autostep;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.swerve.DriveSubsystem;
import frc.robot.ANSIcolors;

public class DriveDistanceStep extends AutoStep {

    DriveSubsystem swerve;
    double xSpeed;
    double ySpeed;
    Pose2d start_pos;
    boolean x_completion;
    boolean y_completion;
    double y_distance;
    double x_distance;

	
	// Maybe add a field realtive boolean so it can be toggled

    public DriveDistanceStep(DriveSubsystem swerve, double x_distance, double y_distance, double xSpeed, double ySpeed) { // Distance is in meters!
        super();
        this.swerve = swerve;
        this.x_distance = x_distance;
        this.y_distance = y_distance;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
    }

    public void Begin() {
        start_pos = swerve.getPose();
        x_completion = false;
        y_completion =  false;
    }

    public void Update() {
        if (!(x_completion && y_completion)) {
            System.out.print(ANSIcolors.RED + "Error_x = " + (swerve.getPose().getX()) + ANSIcolors.R);
            System.out.print("Error_y = " + (swerve.getPose().getY()));
            x_completion = (Math.abs(swerve.getPose().getX() - start_pos.getX()) > Math.abs(x_distance));
            y_completion = (Math.abs(swerve.getPose().getY() - start_pos.getY()) > Math.abs(y_distance));
            if (!(x_completion || y_completion)) {swerve.drive(xSpeed, ySpeed, 0.0, false, true);
            } else if ((!x_completion && y_completion)) {swerve.drive(0.0, ySpeed, 0.0, false, true);
            } else {swerve.drive(xSpeed, 0.0, 0.0, false, true);}
        }

        if (x_completion && y_completion) {
            swerve.drive(0.0, 0.0, 0.0, false, true);
            isDone = true;
        }
    }

}

