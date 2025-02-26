package frc.robot.autostep;

import static edu.wpi.first.units.Units.Value;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.MathE;
import frc.robot.Robot;
import java.util.LinkedList;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autostep.ArmEjectStep;
import frc.robot.autostep.ArmIntakeStep;
import frc.robot.autostep.AutoStep;
import frc.robot.autostep.DriveDistanceStep;
import frc.robot.autostep.LowerAlgaePosStep;
import frc.robot.autostep.MotorMoveStep;
import frc.robot.autostep.NavxTurn;
import frc.robot.autostep.SolenoidStep;
import frc.robot.autostep.SwerveAutoDriveStep;
import frc.robot.autostep.Wait;
import frc.robot.swerve.Constants.OIConstants;
import frc.robot.swerve.DriveSubsystem;

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