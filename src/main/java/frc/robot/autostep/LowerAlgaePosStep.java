package frc.robot.autostep;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.MathE;
import frc.robot.Robot;
import frc.robot.autostep.LowerAlgaePosStep;

public class LowerAlgaePosStep extends AutoStep {

    double setpoint = 0.20;
    Timer timer = new Timer();

    SparkMax armMotorRotate = Robot.armMotorRotate;

    public LowerAlgaePosStep(double setpoint) {
        super();
        this.setpoint = setpoint;
    }
    
    public void Begin() {
        timer.reset(); timer.start();
    }

    public void Update() {
        double encoderVal = armMotorRotate.getAbsoluteEncoder().getPosition() - MathE.TAU;

		double motorOut;

		double angleDeterminedSpeed = Math.cos(encoderVal) * 0.6;

        double armInput = -Robot.armPID.calculate(encoderVal, setpoint);

        motorOut = (-(Robot.hCoefficent * angleDeterminedSpeed)) + armInput;

        if (motorOut < -0.9) {
            motorOut = -0.9;
        } else if (motorOut > 1.0) {
            motorOut = 1.0;
        }

        // if (!Robot.algaeSolenoidState) {
        //     motorOut = 0;
        // }
        
        armMotorRotate.set(motorOut);

        if (timer.get() > 0.6) {
            isDone = true;
        }
    }
}