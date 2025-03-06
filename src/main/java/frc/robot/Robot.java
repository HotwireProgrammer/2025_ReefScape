package frc.robot;

// Imports
import java.util.LinkedList;

import javax.imageio.ImageIO;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autostep.ArmEjectStep;
import frc.robot.autostep.ArmIntakeStep;
import frc.robot.autostep.AutoStep;
import frc.robot.autostep.DriveDistanceStep;
import frc.robot.autostep.LowerAlgaePosAsyncStep;
import frc.robot.autostep.LowerAlgaePosStep;
import frc.robot.autostep.MotorMoveStep;
import frc.robot.autostep.NavxTurn;
import frc.robot.autostep.Print;
import frc.robot.autostep.SolenoidStep;
import frc.robot.autostep.SwerveAutoDriveStep;
import frc.robot.autostep.WaitStep;
import frc.robot.autostep.ZeroHeadingStep;
import frc.robot.swerve.Constants.OIConstants;
import frc.robot.swerve.DriveSubsystem;
import java.awt.image.*;
import java.io.File;
import java.io.IOException;
import java.awt.*;

public class Robot extends TimedRobot {
	private Command autonomousCommand;
	// private RobotContainer robotContainer;
	// private final SendableChooser<Command> autoChooser;


	// Timer
	Timer timer1 = new Timer();
	Timer timer2 = new Timer();
	Timer timer3 = new Timer();
	Timer timer4 = new Timer();
	// Joysticks
	public Joystick operator;
	public Joystick driver;
	public boolean arcadeDrive = false;

	// motors here
	// arm motors relative to pick-up position

	public SparkMax climbingWinch = new SparkMax(21, MotorType.kBrushless);
	public SparkMax climberFlaps = new SparkMax(22, MotorType.kBrushless);

	public static SparkMax armMotorTop = new SparkMax(34, MotorType.kBrushless);
	public static SparkMax armMotorBottom = new SparkMax(35, MotorType.kBrushless);

	public static final PIDController armPID = new PIDController(0.3, 0.0, 0.0);

	public static SparkMax armMotorRotate = new SparkMax(33, MotorType.kBrushless);
	double encoderS;
	double encoderF;
	public boolean climberFlapsState = false;

	public Limelight limelight = new Limelight();

	public enum DriveScale {
		linear, squared, tangent, inverse, cb, cbrt,
	}

	public static DriveSubsystem swerveDrive = new DriveSubsystem();

	// Auto
	public LinkedList<AutoStep> firstAuto;
	public LinkedList<AutoStep> mainAutoTwo;
	public LinkedList<AutoStep> mainAuto;
	public LinkedList<AutoStep> taxiAuto;
	public LinkedList<AutoStep> fourStepSquare;

	public LinkedList<AutoStep> autonomousSelected;
	public int currentAutoStep = 0;

	public String autoSelectKey = "autoMode";

	// 𝐏𝐧𝐞𝐮𝐦𝐚𝐭𝐢𝐜𝐬
	public DoubleSolenoid climberSolenoid;
	public DoubleSolenoid coralSolenoid;
	public DoubleSolenoid algaeSolenoid;

	public Compressor compressor;

	public static DigitalInput limitSwitchOne = new DigitalInput(0);
	public static DigitalInput limitSwitchTwo = new DigitalInput(1);
	public static DigitalInput limitSwitchThree = new DigitalInput(2);
	// public Timer clawStopTimer = new Timer();

	public boolean holding = false;
	public boolean coralSolenoidState = true;
	public boolean climberSolenoidState = false;
	public boolean algaeSolenoidState = false;

	public int climbStep = 0;
	public Timer climberStepTimer;
	public boolean firstClick = false;
	public boolean secondClick = false;

	public final static double hCoefficent = 0.14;

	public float voltComp(float percent) {
		return (float) (12.6 * percent / RobotController.getBatteryVoltage());
	}

	public Robot() {
		super();
		// robotContainer = new RobotContainer();
	}

	public void robotInit() {

		// convertToANSI(loadImage((new File("").getAbsolutePath()) + "/logo.png"));
		
		armPID.enableContinuousInput(0, MathE.TAU);
		SmartDashboard.putNumber("ArmP", armPID.getP());
		SmartDashboard.putNumber("ArmI", armPID.getI());
		SmartDashboard.putNumber("ArmD", armPID.getD());

		compressor = new Compressor(PneumaticsModuleType.REVPH);
		coralSolenoid = new DoubleSolenoid(50, PneumaticsModuleType.REVPH, 1, 0);
		climberSolenoid = new DoubleSolenoid(50, PneumaticsModuleType.REVPH, 2, 3);
		algaeSolenoid = new DoubleSolenoid(50, PneumaticsModuleType.REVPH, 4, 5);

		coralSolenoid.set(Value.kForward);
		climberSolenoid.set(Value.kForward);
		algaeSolenoid.set(Value.kForward);
		coralSolenoidState = true;
		climberSolenoidState = true;
		algaeSolenoidState = true;

		// Limelight
		limelight.SetLight(false);
		limelight.Init();
		SmartDashboard.putNumber(autoSelectKey, 0);

		compressor.enableDigital();

		swerveDrive.Init();

		DriverStation.reportWarning("Successful Initialization", true);
	}


	public void robotPeriodic() {
		// CommandScheduler.getInstance().run();
		// Robot.explode
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
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.65f, 0.0f, 0.0f, 1.025f, false));
		// // firstAuto.add(new NavxTurn(swerveDrive, swerveDrive.m_gyro, -45.0f, 0.0f,
		// // 2.0f));
		// // firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.5f, 0.0f, 0.0f, 0.40f));
		// firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.0f, 0.0f, 0.0f, 1.0f));
		// firstAuto.add(new SwerveAutoDriveStep(swerveDrive, -0.25f, 0.0f, 0.0f, 0.30f));
		// firstAuto.add(new NavxTurn(swerveDrive, swerveDrive.m_gyro, 65.0f, 0.0f, 2.0f));
		// firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.0f, 0.0f, 0.0f, 0.5f));
		// firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.45f, 0.0f, 0.0f, 2.0f));
		// firstAuto.add(new NavxTurn(swerveDrive, swerveDrive.m_gyro, 118.0f, 0.0f, 2.0f));
		// firstAuto.add(new Wait(0.10f));
		// firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.50f, 0.0f, 0.0f, 0.60f));
		// firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.0f, 0.0f, 0.0f, 1.5f));
		// firstAuto.add(new SwerveAutoDriveStep(swerveDrive, -0.50f, 0.0f, 0.0f, 0.60f));
		// firstAuto.add(new NavxTurn(swerveDrive, swerveDrive.m_gyro, 55.0f, 0.0f, 2.0f));
		// firstAuto.add(new SwerveAutoDriveStep(swerveDrive, -0.45f, 0.0f, 0.0f, 2.0f));
		// firstAuto.add(new NavxTurn(swerveDrive, swerveDrive.m_gyro, 1.0f, 0.0f, 2.0f));
		// firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.0f, 0.0f, 0.0f, 0.5f));
		// firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.50f, 0.0f, 0.0f, 0.35f));

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

		mainAutoTwo = new LinkedList<AutoStep>();
		// mainAuto.add(new Print("Running mainAuto"));
		mainAutoTwo.add(new SolenoidStep(coralSolenoid, Value.kReverse));
		mainAutoTwo.add(new MotorMoveStep(armMotorRotate, 0.7f, 0.13f));
		mainAutoTwo.add(new SwerveAutoDriveStep(
			swerveDrive, 
			0.0f, 0.25f, 
			0.0f, 
			0.9f, 
			false));
		mainAutoTwo.add(new LowerAlgaePosAsyncStep(0.42));
		mainAutoTwo.add(new SwerveAutoDriveStep(swerveDrive, 0.25f, 0.0f, 0.0f, 0.90f, false));
		mainAutoTwo.add(new ArmIntakeStep());
		mainAutoTwo.add(new SwerveAutoDriveStep(swerveDrive, 0.0f, -0.15f, 0.0f, 0.75f, false));
		mainAutoTwo.add(new MotorMoveStep(armMotorRotate, 0.0f, 0.0f));
		mainAutoTwo.add(new NavxTurn(swerveDrive, swerveDrive.m_gyro, -87.0f, 0.0f, 2.0f));
		mainAutoTwo.add(new SwerveAutoDriveStep(swerveDrive, 0.18f, 0.0f, 0.0f, 0.75f, false));
		mainAutoTwo.add(new SolenoidStep(coralSolenoid, Value.kForward));
		mainAutoTwo.add(new WaitStep(0.8f));
		mainAutoTwo.add(new SwerveAutoDriveStep(swerveDrive, -0.12f, 0.0f, 0.0f, 0.43f, false));
		mainAutoTwo.add(new SwerveAutoDriveStep(swerveDrive, -0.22f, 0.0f, 0.0f, 3.8f, true));
		mainAutoTwo.add(new ArmEjectStep());
		mainAutoTwo.add(new ZeroHeadingStep());



		fourStepSquare = new LinkedList<AutoStep>();
		fourStepSquare.add(new SwerveAutoDriveStep(swerveDrive, 0.22f, 0.0f, 0.0f, 0.25f, true));
		fourStepSquare.add(new SwerveAutoDriveStep(swerveDrive, 0.0f, 0.22f, 0.0f, 0.25f, true));
		fourStepSquare.add(new SwerveAutoDriveStep(swerveDrive, -0.22f, 0.0f, 0.0f, 0.25f, true));
		fourStepSquare.add(new SwerveAutoDriveStep(swerveDrive, 0.0f, -0.22f, 0.0f, 0.25f, true));
		









		mainAuto = new LinkedList<AutoStep>();

		mainAuto.add(new SolenoidStep(coralSolenoid, Value.kReverse));
		mainAuto.add(new MotorMoveStep(armMotorRotate, 0.55f, 0.15f));
		mainAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.20f, 0.00f, 0.0f, 1.62f, false));
		mainAuto.add(new SolenoidStep(coralSolenoid, Value.kForward));
		mainAuto.add(new WaitStep(0.8f));
		mainAuto.add(new SwerveAutoDriveStep(swerveDrive, -0.15f, 0.0f, 0.0f, 0.82f, false));
		mainAuto.add(new LowerAlgaePosAsyncStep(0.27));
		mainAuto.add(new NavxTurn(swerveDrive, swerveDrive.m_gyro, 86.0f, 0.0f, 2.0f));
		mainAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.0f, 0.20f, 0.0f, 0.75f, false));
		mainAuto.add(new ArmIntakeStep());
		mainAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.0f, -0.15f, 0.0f, 0.75f, false));
		mainAuto.add(new NavxTurn(swerveDrive, swerveDrive.m_gyro, 1.0f, 0.0f, 2.0f));
		mainAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.0f, 0.20f, 0.0f, 4.2f, true));
		mainAuto.add(new ArmEjectStep());
		mainAuto.add(new ZeroHeadingStep());


		taxiAuto = new LinkedList<AutoStep>();
		taxiAuto.add(new SolenoidStep(coralSolenoid, Value.kReverse));
		taxiAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.17f, 0.00f, 0.0f, 0.50f, false));
		

		autonomousSelected = taxiAuto; // Auto Selection

		autonomousSelected.get(0).Begin();
		swerveDrive.zeroHeading();





		// autonomousCommand = robotContainer.getAutonomousCommand();
		// if (autonomousCommand != null) {
		// 	autonomousCommand.schedule();
		// }

	}

	

	public void autonomousPeriodic() {
		// Autonomous-Loop
		if (currentAutoStep < autonomousSelected.size()) {

			autonomousSelected.get(currentAutoStep).Update();

			if (autonomousSelected.get(currentAutoStep).isDone) {
				System.out.println("Step " + currentAutoStep + " done!");
				currentAutoStep += 1;
				if (currentAutoStep < autonomousSelected.size()) {
					autonomousSelected.get(currentAutoStep).Begin();
				}
			}
		} else {
			// Ends Drivetrain Motion.
			swerveDrive.drive(0, 0, 0, false, true);
		}



	}

	public void teleopInit() {
		//String basePath = new File("").getAbsolutePath(); Window window = new Window("Hotwire Robotics", null, 2494, 628); window.addImage(basePath + "logo.png", 1.0, 1247, 314); window.show();

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
		} else if (operator.getRawButton(5)) {// expell
			armMotorTop.set(-0.7);
			armMotorBottom.set(-0.7);
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
		if (operator.getRawButtonPressed(3)) {
			algaeSolenoidState = !algaeSolenoidState;
		}
		if (algaeSolenoidState) {
			algaeSolenoid.set(Value.kForward);
			// System.out.println(algaeSolenoid.get());
		} else {
			algaeSolenoid.set(Value.kReverse);
			// System.out.println(algaeSolenoid.get());
		}

		double axis2 = driver.getRawAxis(2);
		double axis3 = driver.getRawAxis(3);
		if (axis2 >= 0.1) {
			climbingWinch.set(-0.35 * axis2);
		} else if (axis3 >= 0.1) {
			climbingWinch.set(0.35 * axis3);
		} else {
			climbingWinch.set(0);
		}

		// climbing flappies
		

		if (driver.getRawButtonPressed(2)) {
			timer2.reset();
			timer2.start();
			climberFlapsState = true;
			climberFlaps.set(0.4);
		} else if (driver.getRawButtonPressed(4)) {
			timer2.reset();
			timer2.start();
			climberFlapsState = false;
			climberFlaps.set(-0.2);
		}
		if (timer2.get() >= 0.65) {
			climberFlaps.set(0.00);
			// if (climberFlapsState) {
			// 	climberFlaps.set(0.02);
			//  System.out.println("Holding climber flaps");
			// } else {
			// 	climberFlaps.set(-0.02);
			//  System.out.println("Holding climber flaps");
			// } else {
			//  System.out.println("Help, me no work");
			// }
			timer2.stop(); timer2.reset();
		}
		// System.out.println(limitSwitchThree.get());

		// Climber Solenoid (Beak) Holding and release. NO TOUCHY
		if (driver.getRawButton(6)) {
			// if (climberSolenoid.get() == Value.kForward) {
			// 	climberSolenoid.set(Value.kReverse);
			// } else {
			// 	climberSolenoid.set(Value.kForward);
			// }
			climberSolenoid.set(Value.kForward);

		} else if (limitSwitchThree.get()) {
			climberSolenoid.set(Value.kReverse);
			climberFlaps.set(0.00); // stop the climberFlaps from force-holding position.
		}

		armPID.setP(SmartDashboard.getNumber("ArmP", armPID.getP()));
		armPID.setI(SmartDashboard.getNumber("ArmI", armPID.getI()));
		armPID.setD(SmartDashboard.getNumber("ArmD", armPID.getD()));
		

		double encoderVal = armMotorRotate.getAbsoluteEncoder().getPosition() - MathE.TAU;
		// System.out.println("Arm Rotation Encoder = " + encoderVal + "*r");


		// boolean holdAlways = false;



		double angleDeterminedSpeed = Math.cos(encoderVal) * 0.6;
		if ((operator.getRawButton(9) || operator.getRawButton(7))) {
			double motorOut;
			

			double armInput = 0.0;
			if (operator.getRawButton(8)) {
				// todo get arm speed from PID

				armInput = -armPID.calculate(encoderVal, 0.31);
			} else {
				armInput = (operator.getRawAxis(1) / 5);
			}

			if (encoderVal < 0) {
				if (algaeSolenoid.get() == Value.kForward) {
					motorOut = (-(hCoefficent * angleDeterminedSpeed)) + armInput;
				} else {
					motorOut = 0;
				}
				// if (algaeSolenoid.get() != Value.kForward) {
				// 	algaeSolenoid.set(Value.kForward);
				// } else {
				// 	motorOut = (-(hCoefficent * angleDeterminedSpeed)) + armInput;
				// }
			} else {
				motorOut = (-(hCoefficent * angleDeterminedSpeed)) + armInput;
			}

			if (motorOut < -0.9) {
				motorOut = -0.9;
			} else if (motorOut > 1.0) {
				motorOut = 1.0;
			}
			armMotorRotate.set(motorOut);
		} else {
			armMotorRotate.set(0.0);
		}

		// System.out.println(encoderVal);

		/*if ((operator.getRawButton(9) || operator.getRawButton(7))) {
			double encoderVal = armMotorRotate.getAbsoluteEncoder().getPosition() - MathE.TAU;

			double motorOut = 0;

			double angleDeterminedSpeed = Math.cos(encoderVal) * 0.6;

			double armInput = -Robot.armPID.calculate(encoderVal, ((((operator.getRawAxis(1) + 1) / 2) * (1.5 * Math.PI))));

			motorOut = (-(Robot.hCoefficent * angleDeterminedSpeed)) + armInput;

			if (motorOut < -0.9) {
				motorOut = -0.9;
			} else if (motorOut > 1.0) {
				motorOut = 1.0;
			}
			
			armMotorRotate.set(motorOut);
		} else {
			armMotorRotate.set(0.0);
		}*/








		

		// D0 radians/decisecond or millisecond?

		// timer2.reset(); timer2.start();
		// double holdSpeed = 0.10;
		// if (timer2.get() == 0) {
		// encoderF = armMotorRotate.getAbsoluteEncoder().getPosition();
		// } else if (timer2.get() >= 0.01) {
		// encoderS = armMotorRotate.getAbsoluteEncoder().getPosition();
		// }
		// if ((encoderF - encoderS) >= 0) {

		// }
		// armMotorRotate.set(holdSpeed);

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

	public double holdAngle(SparkMax motor, double rad) {
		double motorOutputOut = 0;
		double encoder = motor.getAbsoluteEncoder().getPosition() - MathE.TAU;
		final double kTurningIntensity = 0.5;
		final double kTolerance = MathE.TAU / 70;

		final double kHardLimit = 0.13;


		return motorOutputOut;
	}

	public BufferedImage loadImage(String path) {
		try {
			return ImageIO.read(new File(path));
		} catch (IOException e) {
			e.printStackTrace();
			return null;
		}
	}

	public String convertToANSI(BufferedImage img) {
		String out = "";
		for (int x = 0; x < img.getWidth(); x+=(img.getWidth() / 40)) {for (int y = 0; y < img.getHeight(); y+=(img.getHeight() / 40)) {
			// for every n'th pixel
			if ((x / img.getWidth()) == Math.round(x / img.getWidth())) {out = out + "\n";}
			Color colorPX = new Color(img.getRGB(x, y));
			double mColor = (colorPX.getRed() + colorPX.getGreen() + colorPX.getBlue()) / 3;
			if (mColor >= (0.9 * 255)) {out = out + "&";} 
			else if (mColor >= (0.8 * 255)) {out = out + "$";} 
			else if (mColor >= (0.7 * 255)) {out = out + "X";}
			else if (mColor >= (0.6 * 255)) {out = out + "x";}
			else if (mColor >= (0.5 * 255)) {out = out + "=";}
			else if (mColor >= (0.4 * 255)) {out = out + "+";}
			else if (mColor >= (0.3 * 255)) {out = out + ";";}
			else if (mColor >= (0.2 * 255)) {out = out + ".";}
			else if (mColor >= (0.1 * 255)) {out = out + " ";}
			
		}}

		// &$Xx=+;.
		return out;
	}
}