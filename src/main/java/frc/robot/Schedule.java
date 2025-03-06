package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class Schedule {
    
    Timer timer = new Timer();
    double time;

    public Schedule(double timeInSeconds) {
        timer.reset(); timer.start();
        this.time = timeInSeconds;
    }

    public boolean get() {


        if (timer.get() > time) {
            return true;
        } else {
            return false;
        }
    }
}
