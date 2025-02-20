package frc.robot.autostep;

import com.studica.frc.AHRS;
import frc.robot.swerve.DriveSubsystem;
import frc.robot.c;

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
        System.out.println(c.RED + "navxTurn turnDegree: " + turnDegree + c.RESET);
        float degreeDifference = Math.abs(navx.getYaw() - (turnDegree));
        System.out.println(c.CYAN + "navxTurn botrot: " + targetRotation + c.RESET);
        driveTrain.GoToRotation(turnDegree);

        //float goodEnoughDeg = 5.0f;
        if (degreeDifference < goodEnoughDeg) {
            driveTrain.drive(0, 0, 0, false, true);
            targetRotation += navx.getYaw();
            isDone = true;
        }
    }
}