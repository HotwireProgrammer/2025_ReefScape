package frc.robot.autostep;

public abstract class AutoStep {
    public float botrot = 0.0f;
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