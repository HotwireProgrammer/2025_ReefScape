package frc.robot.autostep;

import frc.robot.Robot;

public class ArmIntakeStep extends AutoStep {

    public ArmIntakeStep() {
        super();
    }
    
    public void Begin() {
        Robot.armMotorBottom.set(0.8);
        Robot.armMotorTop.set(0.8);
        
    }

    public void Update() {

        if (Robot.limitSwitchOne.get() || Robot.limitSwitchTwo.get()) {
            Robot.armMotorBottom.set(0.1);
            Robot.armMotorTop.set(0.1);
            isDone = true;
        }
        
    }
}