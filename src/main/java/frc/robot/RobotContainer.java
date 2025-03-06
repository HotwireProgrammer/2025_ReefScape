// package frc.robot;

// import com.pathplanner.lib.auto.AutoBuilder;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;

// public class RobotContainer {
//     private final SendableChooser<Command> autoChooser;
  
//     public RobotContainer() {
  
//       // Build an auto chooser. This will use Commands.none() as the default option.
//       autoChooser = AutoBuilder.buildAutoChooser();
  
//       // Another option that allows you to specify the default auto by its name
//       // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
  
//       SmartDashboard.putData("Auto Chooser", autoChooser);
//     }
  
//     public Command getAutonomousCommand() {
//       return autoChooser.getSelected();
//     }
// }