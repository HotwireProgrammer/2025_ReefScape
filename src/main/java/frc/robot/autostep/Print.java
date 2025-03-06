package frc.robot.autostep;

public class Print extends AutoStep {

    public String out;

    public Print(String out) {
        super();
        this.out = out;
    }
    
    public void Begin() {
        isDone = true;
        new Thread(() -> {
            System.out.println(out);
        }).start();
    }

    public void Update() {
        
    }
}