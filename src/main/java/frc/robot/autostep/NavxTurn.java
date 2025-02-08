package frc.robot.autostep;

import com.studica.frc.AHRS;
import frc.robot.swerve.DriveSubsystem;
import frc.robot.ANSIcolors;

public class NavxTurn extends AutoStep {

    public AHRS navx;
    public float turnDegree;
    public float speed;
    public float goodEnoughDeg;
    public float startDegree;

    public DriveSubsystem driveTrain;

    public NavxTurn(DriveSubsystem driveTrain, AHRS navx, float turnDegree, float speed, float goodEnoughDeg) {
        super();
        this.navx = navx;
        this.speed = speed;
        this.turnDegree = turnDegree;
        this.driveTrain = driveTrain;
        this.goodEnoughDeg = goodEnoughDeg;
    }
    
    public void Begin() {
        
     }

    public void Update() {
        System.out.println(ANSIcolors.RED + "navxTurn turnDegree: " + turnDegree + ANSIcolors.RESET);
        float degreeDifference = Math.abs(navx.getYaw() - (turnDegree));
        System.out.println(ANSIcolors.CYAN + "navxTurn botrot: " + botrot + ANSIcolors.RESET);
        driveTrain.GoToRotation(turnDegree);

        //float goodEnoughDeg = 5.0f;
        if (degreeDifference < goodEnoughDeg) {
            driveTrain.drive(0, 0, 0, false, true);
            botrot = navx.getYaw();
            isDone = true;
        }
    }
}