package frc.robot.autostep;

public abstract class AutoStep {
    public double targetRotation = 0.0;
    public boolean isDone;

    public boolean autoIndex = true;
    public boolean runShooter = false;

    public AutoStep()
    {
        isDone = false;
    }

    public abstract void Begin();
    public abstract void Update();
}