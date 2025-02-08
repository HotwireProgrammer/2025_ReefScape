package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.autostep.AutoStep;
import frc.robot.autostep.SwerveAutoDriveStep;
import frc.robot.swerve.DriveSubsystem;

import java.util.LinkedList;
import frc.robot.autostep.SwerveAutoDriveStep;
import frc.robot.Robot;

public class Auto {
    
    // Remember, don't use PH port 0 (or maybe 1 either)
    int PHreverseChannel; // This will be the port on your PH for reverse solenoid movement.
    int PHforwardChannel; // This will be the port on your PH for forward solenoid movement.
    
    LinkedList<AutoStep> auto;
    boolean solenoidState = false;
    int PHid = 50; // This will be your Pneumatic Hub ID

    public Auto(LinkedList<AutoStep> AutoStep) {
        auto.add(new SwerveAutoDriveStep(new DriveSubsystem(), 0, 0, 0, 0));
    }
}