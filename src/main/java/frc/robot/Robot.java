package frc.robot;

// Imports
import java.util.LinkedList;

import javax.print.FlavorException;

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
import frc.robot.autostep.NavxTurn;
import frc.robot.autostep.SwerveAutoDriveStep;
import frc.robot.autostep.Wait;
import frc.robot.swerve.Constants.OIConstants;
import frc.robot.swerve.DriveSubsystem;

public class Robot extends TimedRobot {

	// Timer
	Timer timer1 = new Timer();
	Timer timer2 = new Timer();
	// Joysticks
	public Joystick operator;
	public Joystick driver;
	public boolean arcadeDrive = false;

	// motors here
	// arm motors relative to pick-up position

	public SparkMax climbingWinch = new SparkMax(21, MotorType.kBrushless);
	public SparkMax climberFlaps = new SparkMax(22, MotorType.kBrushless);

	public SparkMax armMotorTop = new SparkMax(31, MotorType.kBrushless);
	public SparkMax armMotorBottom = new SparkMax(32, MotorType.kBrushless);
	public SparkMax armMotorRotate = new SparkMax(33, MotorType.kBrushless);

	public boolean climberFlapsState = false;
	public boolean DesiredclimberFlapsState = false;

	public Limelight limelight = new Limelight();

	public enum DriveScale {
		linear, squared, tangent, inverse, cb, cbrt,
	}

	public DriveSubsystem swerveDrive = new DriveSubsystem();

	// Auto
	public LinkedList<AutoStep> firstAuto;
	public LinkedList<AutoStep> testAuto;

	public LinkedList<AutoStep> autonomousSelected;
	public int currentAutoStep = 0;

	public String autoSelectKey = "autoMode";

	// 𝐏𝐧𝐞𝐮𝐦𝐚𝐭𝐢𝐜𝐬
	public DoubleSolenoid climberSolenoid;
	public DoubleSolenoid coralSolenoid;
	public DoubleSolenoid algaeSolenoid;

	public Compressor compressor;

	public DigitalInput limitSwitchOne = new DigitalInput(0);
	public DigitalInput limitSwitchTwo = new DigitalInput(1);
	public DigitalInput limitSwitchThree = new DigitalInput(2);
	// public Timer clawStopTimer = new Timer();

	public boolean holding = false;
	public boolean coralSolenoidState = false;
	public boolean climberSolenoidState = false;
	public boolean algaeSolenoidState = false;

	public int climbStep = 0;
	public Timer climberStepTimer;
	public boolean firstClick = false;
	public boolean secondClick = false;

	public double hCoefficent = 0.13;

	public float voltComp(float percent) {
		return (float) (12.6 * percent / RobotController.getBatteryVoltage());
	}

	public Robot() {
		super();
	}

	public void robotInit() {
		System.out.println(ANSIcolors.PURPLE + "Success!" + ANSIcolors.RESET);
		compressor = new Compressor(PneumaticsModuleType.REVPH);
		coralSolenoid = new DoubleSolenoid(50, PneumaticsModuleType.REVPH, 1, 0);
		climberSolenoid = new DoubleSolenoid(50, PneumaticsModuleType.REVPH, 3, 2);
		algaeSolenoid = new DoubleSolenoid(50, PneumaticsModuleType.REVPH, 4, 5);

		// Limelight
		limelight.SetLight(false);
		limelight.Init();
		SmartDashboard.putNumber(autoSelectKey, 0);

		compressor.enableDigital();

		swerveDrive.Init();
	}

	public void disabledInit() {
		// 𝐂𝐎𝐍𝐓𝐑𝐎𝐋𝐋𝐄𝐑𝐒 🎮
		driver = new Joystick(1);
		operator = new Joystick(2);
	}

	public void disabledPeriodic() {

	}

	public void autonomousInit() {
		currentAutoStep = 0;

		firstAuto = new LinkedList<AutoStep>();
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.65f, 0.0f, 0.0f, 1.025f));
		// firstAuto.add(new NavxTurn(swerveDrive, swerveDrive.m_gyro, -45.0f, 0.0f,
		// 2.0f));
		// firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.5f, 0.0f, 0.0f, 0.40f));
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.0f, 0.0f, 0.0f, 1.0f));
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, -0.25f, 0.0f, 0.0f, 0.30f));
		firstAuto.add(new NavxTurn(swerveDrive, swerveDrive.m_gyro, 65.0f, 0.0f, 2.0f));
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.0f, 0.0f, 0.0f, 0.5f));
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.45f, 0.0f, 0.0f, 2.0f));
		firstAuto.add(new NavxTurn(swerveDrive, swerveDrive.m_gyro, 118.0f, 0.0f, 2.0f));
		firstAuto.add(new Wait(0.10f));
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.50f, 0.0f, 0.0f, 0.60f));
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.0f, 0.0f, 0.0f, 1.5f));
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, -0.50f, 0.0f, 0.0f, 0.60f));
		firstAuto.add(new NavxTurn(swerveDrive, swerveDrive.m_gyro, 55.0f, 0.0f, 2.0f));
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, -0.45f, 0.0f, 0.0f, 2.0f));
		firstAuto.add(new NavxTurn(swerveDrive, swerveDrive.m_gyro, 1.0f, 0.0f, 2.0f));
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.0f, 0.0f, 0.0f, 0.5f));
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.50f, 0.0f, 0.0f, 0.35f));

		// Turning test EXPERIMENTAL
		// firstAuto.add(new NavxTurn(swerveDrive, swerveDrive.m_gyro, (float)
		// limelight.GetX(), 0.0f, 1.0f));

		// firstAuto.add(new SolenoidStep(coralSolenoid, Value.kForward));
		// firstAuto.add(new Wait(1.0f));
		// firstAuto.add(new SolenoidStep(coralSolenoid, Value.kReverse));
		// firstAuto.add(new SwerveAutoDriveStep(swerveDrive, -0.25f, 0.0f, 0.0f,
		// 1.0f));
		// firstAuto.add(new NavxTurn(swerveDrive, swerveDrive.m_gyro, 45.0f, 0.0f,
		// 1.0f));
		// firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.25f, 0.0f, 0.0f, 1.5f));
		// firstAuto.add(new NavxTurn(swerveDrive, swerveDrive.m_gyro, -45.0f, 0.0f,
		// 1.0f));
		// firstAuto.add(new Wait(1.5f));

		testAuto = new LinkedList<AutoStep>(); // Attempt once ⭐ method in 'DriveSubsystem' is implemented.
		for (int I585 = 0; I585 < 5; I585++) {
			testAuto.add(new NavxTurn(swerveDrive, swerveDrive.m_gyro, (float) limelight.GetX(), 0.0f, 0.4f));
			testAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.3f, 0.0f, 0.0f, 0.25f));
		}

		autonomousSelected = testAuto; // Auto Selection

		autonomousSelected.get(0).Begin();
		swerveDrive.zeroHeading();
	}

	public void autonomousPeriodic() {
		// autonomous loop

		// System.out.println(ANSIcolors.PURPLE + "Current auto step " + currentAutoStep
		// + ANSIcolors.RESET);
		if (currentAutoStep < autonomousSelected.size()) {

			autonomousSelected.get(currentAutoStep).Update();

			if (autonomousSelected.get(currentAutoStep).isDone) {
				currentAutoStep += 1;
				if (currentAutoStep < autonomousSelected.size()) {
					autonomousSelected.get(currentAutoStep).Begin();
				}
			}
		} else {
			// Stops the drivetrain.
			swerveDrive.drive(0, 0, 0, false, true);
		}

	}

	public void teleopInit() {
		SmartDashboard.putNumber("Hold Coefficent", hCoefficent);

		limelight.SetLight(false);

		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);

		// 𝐂𝐎𝐍𝐓𝐑𝐎𝐋𝐋𝐄𝐑𝐒 🎮
		driver = new Joystick(1);
		operator = new Joystick(2);
	}

	public void teleopPeriodic() {

		// 𝐃𝐑𝐈𝐕𝐄𝐑 𝐂𝐎𝐍𝐓𝐑𝐎𝐋𝐒
		double pow = 2;
		double a0 = driver.getRawAxis(0);
		double a1 = driver.getRawAxis(1);
		double axisZero = Math.pow(a0, pow) * Math.signum(a0);
		double axisOne = Math.pow(a1, pow) * Math.signum(a1); // ε

		if (driver.getRawButton(6)) {
			axisZero = axisZero / 4;
			axisOne = axisOne / 4;
		}

		// 𝐀𝐑𝐌 𝐌𝐎𝐓𝐎𝐑 𝐋𝐎𝐆𝐈𝐂
		if (operator.getRawButton(6)) {
			// Limit-switches return 𝘁𝗿𝘂𝗲 when blocked!
			if (!(limitSwitchOne.get() || limitSwitchTwo.get())) { // L1 nor L2
				// grab
				armMotorTop.set(0.8);
				armMotorBottom.set(0.8);
				// algaeSolenoidState = true;
			} else {
				// hold
				armMotorTop.set(0.1);
				armMotorBottom.set(0.1);
				// algaeSolenoidState = false;
			}
		} else if (operator.getRawButton(3)) {// expell
			armMotorTop.set(-0.8);
			armMotorBottom.set(-0.8);
		} else {// default
			armMotorTop.set(0.0);
			armMotorBottom.set(0.0);
		}

		// 𝐒𝐎𝐋𝐄𝐍𝐎𝐈𝐃 𝐒𝐖𝐈𝐓𝐂𝐇𝐄𝐒

		// Value v = coralSolenoid.get();
		// SmartDashboard.putString("solenoid", v.toString());

		// coral solenoid
		if (operator.getRawButtonPressed(1)) {
			coralSolenoidState = !coralSolenoidState;
		}
		if (coralSolenoidState) {
			coralSolenoid.set(Value.kReverse);
		} else {
			coralSolenoid.set(Value.kForward);
		}

		// algae solenoid
		if (operator.getRawButtonPressed(5)) {
			algaeSolenoidState = !algaeSolenoidState;
		}
		if (algaeSolenoidState) {
			algaeSolenoid.set(Value.kForward);
		} else {
			algaeSolenoid.set(Value.kReverse);
		}

		double axis2 = operator.getRawAxis(2);
		double axis3 = operator.getRawAxis(3);
		if (axis2 >= 0.1) {
			climbingWinch.set(0.25 * axis2);
		} else if (axis3 >= 0.1) {
			climbingWinch.set(-0.25 * axis3);
		} else {
			climbingWinch.set(0);
		}

		// climbing flappies
		if (driver.getRawButtonPressed(2)) {
			timer2.reset();
			timer2.start();
			climberFlaps.set(0.10);
		} else if (driver.getRawButtonPressed(4)) {
			timer2.reset();
			timer2.start();
			climberFlaps.set(-0.10);
		}
		if (timer2.get() >= 0.75) {
			climberFlaps.set(0.00);
			timer2.stop();
		}

		// Climber Solenoid (Beak) Holding and release. NO TOUCHY
		if (operator.getRawButton(4)) {

			climberSolenoid.set(Value.kReverse);

		} else {

			if (limitSwitchThree.get()) {
				climberSolenoid.set(Value.kForward);
			} else {
				climberSolenoid.set(Value.kReverse);
			}
		}

		double encoderVal = armMotorRotate.getAbsoluteEncoder().getPosition();
		double angleDeterminedSpeed = Math.cos(encoderVal);
		if (operator.getRawButton(7)) {
			double motorOut = (-(hCoefficent * angleDeterminedSpeed)) + (operator.getRawAxis(1) / 10);

			if (motorOut < -0.9) {
				motorOut = -0.9;
			} else if (motorOut > 1.0) {
				motorOut = 1.0;
			}
			System.out.println(motorOut);
			armMotorRotate.set(motorOut);

		} else {
			armMotorRotate.set(0);
		}

		swerveDrive.drive(
				-MathUtil.applyDeadband(axisOne, OIConstants.kDriveDeadband), // 0.05
				-MathUtil.applyDeadband(axisZero, OIConstants.kDriveDeadband), // 0.05
				-MathUtil.applyDeadband(driver.getRawAxis(4), OIConstants.kDriveDeadband), // 0.05
				true, true);

		// zero
		if (driver.getRawButton(1)) {
			swerveDrive.zeroHeading();
		}

		// the fist of khonshu..

		// the MOON..

		// hCoefficent = SmartDashboard.getNumber("Hold Coefficent", hCoefficent);

	}

	public float DriveScaleSelector(float ControllerInput, DriveScale selection) {

		float multiplier = (ControllerInput / (float) Math.abs(ControllerInput));
		// float multiplier = Math.signum(ControllerInput);
		if (selection == DriveScale.squared) {
			float output = multiplier * (float) (Math.pow(ControllerInput, 2));

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
		SmartDashboard.putNumber("navx turn value", swerveDrive.m_gyro.getYaw());
		SmartDashboard.putNumber("navx get angle", swerveDrive.m_gyro.getAngle());
		SmartDashboard.putNumber("navx get pitch", swerveDrive.m_gyro.getPitch());
		swerveDrive.drive(
				-MathUtil.applyDeadband(driver.getRawAxis(0), OIConstants.kDriveDeadband), // 0.05
				MathUtil.applyDeadband(driver.getRawAxis(1), OIConstants.kDriveDeadband), // 0.05
				-MathUtil.applyDeadband(driver.getRawAxis(0), OIConstants.kDriveDeadband), // 0.05
				true, false);
	}

	public static float Lerp(float v0, float v1, float t) {
		/*
		 * t = Math.abs(t);
		 * if (t > 1) {t = 1;}
		 * return (v0 + t * (v1 - v0));
		 */
		if (t < 0) {
			t = 0;
		} else if (t > 1) {
			t = 1;
		}
		return (v0 + t * (v1 - v0));
	}

	public float TranslateController(float input) {
		float deadzone = 0.15f;
		if ((input > -deadzone) && (input < deadzone)) {
			input = 0.0f;
		}
		float a = 0.7f;
		float output = (a * ((float) Math.pow(input, 3))) + (1 - a) * input;
		return output;
	}
}