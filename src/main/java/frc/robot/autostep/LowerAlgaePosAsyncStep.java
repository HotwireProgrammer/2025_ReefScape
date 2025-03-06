package frc.robot.autostep;

import static edu.wpi.first.units.Units.Value;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.MathE;
import frc.robot.Robot;


public class LowerAlgaePosAsyncStep extends AutoStep {

    public double setpoint = 0.27;
    Timer timer = new Timer();

    SparkMax armMotorRotate = Robot.armMotorRotate;

    public LowerAlgaePosAsyncStep(double setpoint) {
        super();
        this.setpoint = setpoint; // might work now
    }
    
    public void Begin() {
        timer.reset(); timer.start();
        new Thread(() -> {
            while (true) {
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

                // if ((Robot.algaeSolenoid.get() == Value.kReverse)) {
                //     motorOut = 0;
                // }
                
                armMotorRotate.set(motorOut);

                if (Robot.limitSwitchTwo.get() || Robot.limitSwitchOne.get() || (timer.get() > 2.0)) {
                    armMotorRotate.set(0);
                    break;
                }
            }
        }).start();

        
        isDone = true;
    }

    public void Update() {

    }
}