package frc.robot;

import java.util.LinkedList;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autostep.AutoStep;
import frc.robot.autostep.LimelightTrack;
import frc.robot.autostep.NavxTurn;
import frc.robot.autostep.SwerveAutoDriveStep;
import frc.robot.swerve.Constants.OIConstants;
import frc.robot.swerve.DriveSubsystem;

public class Robot extends TimedRobot {

	// Joysticks
	public Joystick operator;
	public boolean arcadeDrive = false;
	public Joystick driver;

	// motors here

	public SparkMax climbingWinch = new SparkMax(20, MotorType.kBrushless);
	public SparkMax armMotorTop = new SparkMax(30, MotorType.kBrushless);
	public SparkMax armMotorBottom = new SparkMax(31, MotorType.kBrushless);

	public Limelight limelight = new Limelight();

	public enum DriveScale {
		linear, squared, tangent, inverse, cb, cbrt,
	}

	public DriveSubsystem swerveDrive = new DriveSubsystem();

	// Auto
	public LinkedList<AutoStep> firstAuto;

	public LinkedList<AutoStep> autonomousSelected;
	public int currentAutoStep = 0;

	public String autoSelectKey = "autoMode";

	// 𝐏𝐧𝐞𝐮𝐦𝐚𝐭𝐢𝐜𝐬
	public DoubleSolenoid climberSolenoid;
	public DoubleSolenoid coralSolenoid;

	public Compressor compressor;

	public DigitalInput limitSwitchOne = new DigitalInput(0);
	// public DigitalInput limitSwitchTwo = new DigitalInput(0);
	// public Timer clawStopTimer = new Timer();

	public boolean holding = false; // 🫵
	public boolean coralSolenoidState = false; // 🪸
	public boolean climberSolenoidState = false;

	// public boolean climberSolenoidState = false;

	public int climbStep = 0;
	public Timer climberStepTimer;
	public boolean firstClick = false;
	public Boolean secondClick = false;

	public float voltComp(float percent) {
		return (float) (12.6 * percent / RobotController.getBatteryVoltage());
	}

	public Robot() {
		super();
	}

	public void robotInit() {

		compressor = new Compressor(PneumaticsModuleType.REVPH);
		coralSolenoid = new DoubleSolenoid(50, PneumaticsModuleType.REVPH, 0, 1);
		// climberSolenoid = new DoubleSolenoid(50, PneumaticsModuleType.REVPH, 2, 3);

		limelight.SetLight(false);
		limelight.Init();
		SmartDashboard.putNumber(autoSelectKey, 0);
		compressor.enableDigital();

		swerveDrive.Init();
	}

	public void disabledInit() {

		// Controllers
		operator = new Joystick(2);
		driver = new Joystick(1);
	}

	public void disabledPeriodic() {

	}

	public void autonomousInit() {
		currentAutoStep = 0;

		firstAuto = new LinkedList<AutoStep>();
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.25f, 0, 0, 1.0f));
		firstAuto.add(new LimelightTrack(swerveDrive, null, limelight));
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, -0.25f, 0, 0, 1.0f));
		firstAuto.add(new NavxTurn(swerveDrive, swerveDrive.m_gyro, 90, 0, 1));

		autonomousSelected = firstAuto;
		autonomousSelected.get(0).Begin();
		swerveDrive.zeroHeading();
	}

	public void autonomousPeriodic() {

		// autonomous loop
		// System.out.println("Current auto step " + currentAutoStep);
		if (currentAutoStep < autonomousSelected.size()) {

			autonomousSelected.get(currentAutoStep).Update();

			if (autonomousSelected.get(currentAutoStep).isDone) {
				currentAutoStep = currentAutoStep + 1;
				if (currentAutoStep < autonomousSelected.size()) {
					autonomousSelected.get(currentAutoStep).Begin();
				}
			}
		} else {
			// stop drivetrain
			swerveDrive.drive(0, 0, 0, false, true);
		}

	}

	public void teleopInit() {

		limelight.SetLight(false);

		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

		// Controllers
		driver = new Joystick(1);
		operator = new Joystick(2);

	}

	public void teleopPeriodic() {
		// drive controls
		// n^pow
		double pow = 2;
		double axisZero = Math.pow(driver.getRawAxis(0), pow)
				* (driver.getRawAxis(0) / Math.abs(driver.getRawAxis(0)));
		double axisOne = Math.pow(driver.getRawAxis(1), pow)
				* (driver.getRawAxis(1) / Math.abs(driver.getRawAxis(1)));

		if (driver.getRawButton(6)) {
			axisZero = axisZero * 0.25;
			axisOne = axisOne * 0.25;
		}

		if (operator.getRawButton(2)) {
			climbingWinch.set(0.5); // Motor doesn't exist
		}

		if (operator.getRawButton(6)) {
			if (!limitSwitchOne.get()) {
				armMotorTop.set(0.8); // 𝐈𝐃 30
				// The bottom motor is also positive because the motor is inverted!
				armMotorBottom.set(0.8);// 𝐈𝐃 31
			} else {
				armMotorTop.set(0.1); // 𝐈𝐃 30
				// The bottom motor is also positive because the motor is inverted!
				armMotorBottom.set(0.1);// 𝐈𝐃 31
			}
		} else if (operator.getRawButton(3)) {
			// zero speeds
			armMotorTop.set(-0.8);
			armMotorBottom.set(-0.8);
		} else {
			// zero speeds
			armMotorTop.set(0.0);
			armMotorBottom.set(0.0);
		}

		// 𝐒𝐎𝐋𝐄𝐍𝐎𝐈𝐃 𝐒𝐖𝐈𝐓𝐂𝐇𝐄𝐒

		// Value v = coralSolenoid.get();
		// SmartDashboard.putString("solenoid", v.toString());

		// Toggle for the solenoid controlling the coral mechanism
		if (operator.getRawButtonPressed(1)) {
			coralSolenoidState = !coralSolenoidState;
		}

		if (coralSolenoidState) {
			coralSolenoid.set(Value.kForward);
		} else {
			coralSolenoid.set(Value.kReverse);
		}

		// Value v = climberSolenoid.get();
		// SmartDashboard.putString("solenoid", v.toString());

		// Toggle for the solenoid controlling the climbing hooks

		// if (operator.getRawButtonPressed(4))
		// {
		// if (climberSolenoidState == true) {
		// climberSolenoidState = false;
		// }
		// else
		// {
		// climberSolenoidState = true;
		// }
		// }

		// if (climberSolenoidState == true) {
		// climberSolenoid.set(Value.kForward);
		// } else {
		// climberSolenoid.set(Value.kReverse);
		// }

		swerveDrive.drive(
				-MathUtil.applyDeadband(axisOne, OIConstants.kDriveDeadband),
				-MathUtil.applyDeadband(axisZero, OIConstants.kDriveDeadband),
				-MathUtil.applyDeadband(driver.getRawAxis(4), OIConstants.kDriveDeadband),
				true, true);

		// zero
		if (driver.getRawButton(1)) {
			swerveDrive.zeroHeading();

		}
	}

	public float DriveScaleSelector(float ControllerInput, DriveScale selection) {

		float multiplier = (ControllerInput / (float) Math.abs(ControllerInput));

		if (selection == DriveScale.squared) {
			float output = multiplier * (float) (ControllerInput * ControllerInput);

			return output;

		} else if (selection == DriveScale.tangent) {
			return multiplier * (0.4f * (float) Math.tan(1.8 * (multiplier * ControllerInput) - .9) + 0.5f);
		} else if (selection == DriveScale.inverse) {
			return (float) Math.pow(ControllerInput, 1 / 2);
		} else if (selection == DriveScale.cb) {
			return (float) Math.pow(ControllerInput, 3);
		} else if (selection == DriveScale.cbrt) {
			return multiplier * (0.63f * (float) Math.cbrt((multiplier * ControllerInput) - 0.5f) + 0.5f);
		} else {
			return ControllerInput;
		}
	}

	public void testInit() {
		operator = new Joystick(2);
		driver = new Joystick(1);

		swerveDrive.zeroHeading();
	}

	public void testPeriodic() {
		swerveDrive.drive(
				-MathUtil.applyDeadband(driver.getRawAxis(0), OIConstants.kDriveDeadband),
				MathUtil.applyDeadband(driver.getRawAxis(1), OIConstants.kDriveDeadband),
				-MathUtil.applyDeadband(driver.getRawAxis(0), OIConstants.kDriveDeadband),
				true, false);
	}

	public static float Lerp(float v0, float v1, float t) {

		if (t < 0) {
			t = 0;

		} else if (t > 1) {
			t = 1;
		}

		return (v0 + t * (v1 - v0));
	}

	public float TranslateController(float input) {
		float deadzone = 0.15f;
		if (input > -deadzone && input < deadzone) {
			input = 0.0f;
		}
		float a = 0.7f;
		float output = (a * input * input * input) + (1 - a) * input;
		return output;
	}

}

// ⣿⣿⣿⣿⣿⣿⣿⢿⠟⠛⠿⠻⠿⠿⠟⠿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⣿⣿⡿⠛⢙⣨⣥⣶⣶⣿⢿⣿⣿⣷⣦⣅⠛⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⣿⠟⢀⡴⠟⠋⢉⣀⣠⣤⣤⣤⣀⠉⠻⣿⣧⡈⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⣿⠀⠁⣠⣴⣾⣿⣿⣿⣿⣿⣿⣿⣷⠀⢻⣿⣇⠝⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⣿⠀⣼⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⡀⣼⡿⠟⠀⠙⣛⣬⠱⣿⣿⣿⣿⣿⣿
// ⣿⣿⠀⠹⣿⣿⣿⣿⣿⣿⣿⣿⠿⠋⢀⠄⠁⣠⣶⣾⣿⣿⣿⡆⣼⣿⣿⣿⣿⣿
// ⣿⣿⠀⣀⠙⣛⣛⣻⠛⠋⣉⣢⣤⣾⠃⣰⡄⠸⣿⣿⣿⣿⣿⣷⠘⣿⣿⣿⣿⣿
// ⣿⣿⣤⢹⣷⣶⣶⣶⣾⣿⣿⣿⣿⣿⡄⠸⣷⠀⢻⣿⣿⡿⠟⠛⠡⣿⣿⣿⣿⣿
// ⣿⣿⣿⠄⢻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⠄⠻⠇⢈⠁⠀⠀⠲⠠⠞⠿⣿⣿⣿⣿
// ⣿⣿⣿⣷⠈⢿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣶⣶⢤⠀⠀⢲⣿⣿⣿⣷⣤⡉⣻⣿⣿
// ⣿⣿⣿⣿⣧⠈⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣳⡀⢻⣿⣿⣿⣿⣷⠐⣿⣿
// ⣿⣿⣿⣿⣿⣯⡈⢻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣾⡇⡆⣿⣿⣿⣿⡟⣀⣿⣿
// ⣿⣿⣿⣿⣿⣿⣷⡀⢻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠃⢃⡿⠿⠛⡋⣀⣾⣿⣿
// ⣿⣿⣿⣿⣿⣿⣿⣷⣀⠹⣿⣿⣿⣿⣿⣿⣿⠿⠋⢁⣠⣿⡦⠐⠀⢈⡙⢿⣿⣿
// ⣿⣿⣿⣿⣿⣿⣿⣿⠋⢀⣿⣿⣿⣿⠟⢃⣤⣤⡀⠻⣿⣇⣠⣴⡿⠄⠹⣧⡸⣿
// ⣿⣿⣿⣿⣿⣿⡿⠃⢠⣾⣿⣿⡿⢋⣤⣿⣿⣿⣿⣄⠈⢿⡿⠋⣠⣤⣀⠈⣡⣿
// ⣿⣿⣿⠅⣀⣈⠁⣰⣿⣿⡿⠋⣤⣾⣿⣿⣿⣿⣿⣿⣷⣵⣂⣽⣿⣿⣿⣿⣿⣿
// ⣿⣿⣿⣄⠘⢿⣿⣿⠟⠋⣠⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⣿⣿⣿⣷⣤⣬⣅⣶⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿

// yam yam
