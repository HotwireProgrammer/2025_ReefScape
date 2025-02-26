package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitch {


    public LimitSwitch(int channel) {
        DigitalInput ls = new DigitalInput(channel);
    }
}
