package frc.robot.autostep;

import frc.robot.Robot;

public class ZeroHeadingStep extends AutoStep {

    public ZeroHeadingStep() {
        super();
    }
    
    public void Begin() {
        
    }

    public void Update() {
        Robot.swerveDrive.zeroHeading();
        System.out.println("Zeroing Heading");
        isDone = true;
    }
}