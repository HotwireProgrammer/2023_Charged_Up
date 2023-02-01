package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.wpilibj.DriverStation;

import java.applet.AudioClip;
import java.nio.Buffer;
import java.rmi.server.Operation;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
// import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.autostep.*;
import edu.wpi.first.wpilibj.Compressor;
import java.util.*;

//import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {

	// Sensors
	public AnalogGyro headinGryo = new AnalogGyro(0);

	// Drive train
	public DriveTrain driveTrain = new DriveTrain(0, 1, 2, 3);
	// public TalonSRX frontLeft = new TalonSRX(54);
	// public TalonSRX backLeft = new TalonSRX(55);
	// public TalonSRX frontRight = new TalonSRX(56);
	// public TalonSRX backRight = new TalonSRX(57);

	// Logic
	public float pwrm = 1;
	public double setPointArm = 0;

	// Robot Components
	public Gripper gripper;
	public Arm arm = new Arm();

	// Joysticks
	// public Joystick driver;
	public Joystick operator;
	public boolean arcadeDrive = false;
	public Joystick flightStickLeft;
	public Joystick flightStickRight;

	public Limelight limelight = new Limelight();

	public HotPID navxPID = new HotPID("navx", 0.01, 0.01, 0.0);// was 0.0005

	public enum DriveScale {
		linear, squared, tangent, inverse, cb, cbrt,
	}

	public DriverStation driverStation;
	public RobotState currentState;

	// Auto
	public LinkedList<AutoStep> firstAuto;
	public LinkedList<AutoStep> autonomousSelected;
	public int currentAutoStep = 0;

	public String autoSelectKey = "autoMode";

	public float voltComp(float percent) {
		return (float) (12.6 * percent / RobotController.getBatteryVoltage());
	}

	public void robotInit() {
		SmartDashboard.putNumber("Ballcount", 0);
		SmartDashboard.putBoolean("TwoBall", false);
		SmartDashboard.putBoolean("FourBallBlue", false);
		SmartDashboard.putBoolean("FourBallRed", false);
		SmartDashboard.putBoolean("Test", false);
		CameraServer.startAutomaticCapture();
		limelight.SetLight(false);
		limelight.Init();
		SmartDashboard.putNumber(autoSelectKey, 0);

		gripper = new Gripper(this);
	}

	public void periodic() {
		var heading = Rotation2d.fromDegrees(headinGryo.getAngle());
	}

	public void disabledInit() {

		// Controllers
		driveTrain.SetBreak();
		// driver = new Joystick(0);
		operator = new Joystick(1);
		flightStickLeft = new Joystick(3);
		flightStickRight = new Joystick(2);
	}

	public void disabledPeriodic() {
		driveTrain.SendData();
		SmartDashboard.putBoolean("RobotEnabled", false);
	}

	public void autonomousInit() {
		currentAutoStep = 0;
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(1);

		driveTrain.SetBreak();
		limelight.SetLight(true);

		// Autonomous selection

		double autoChoice = SmartDashboard.getNumber(autoSelectKey, 0);
		boolean Test = SmartDashboard.getBoolean("Test", false);

		//firstAuto.add(new TimedForward(driveTrain, 1, 0.2f));
		firstAuto = new LinkedList<AutoStep>();

		firstAuto.add(new ArmMove(arm, 1.5f, 0.1f));

		autonomousSelected = firstAuto;
		autonomousSelected.get(0).Begin();
	}

	public void autonomousPeriodic() {
		SmartDashboard.putBoolean("RobotEnabled", true);

		// autonomous loop
		System.out.println("Current auto step " + currentAutoStep);
		if (currentAutoStep < autonomousSelected.size()) {

			autonomousSelected.get(currentAutoStep).Update();

			if (autonomousSelected.get(currentAutoStep).isDone) {
				currentAutoStep = currentAutoStep + 1;
				if (currentAutoStep < autonomousSelected.size()) {
					autonomousSelected.get(currentAutoStep).Begin();
				}
			}
		} else {
			driveTrain.SetBothSpeed(0.0f);
			// currentState = RobotState.Teleop;
		}

		UpdateMotors();
	}

	public void teleopInit() {

		// navx.reset();
		limelight.SetLight(false);

		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

		driveTrain.SetCoast();

		// Controllers
		// driver = new Joystick(0);
		operator = new Joystick(1);
		flightStickLeft = new Joystick(3);
		flightStickRight = new Joystick(2);

	}

	// Drive Scale
	boolean slow = false;

	public float DriveScaleSelector(float ControllerInput, DriveScale selection) {

		float multiplier = (ControllerInput / (float) Math.abs(ControllerInput));

		if (selection == DriveScale.squared) {
			float output = multiplier * (float) (ControllerInput * ControllerInput);

			if (flightStickLeft.getRawButtonPressed(1)) {
				slow = !slow;
			}

			if (slow) {
				output = output * 0.5f;
			}

			System.out.println(output + "meow");
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

	public void teleopPeriodic() {

		gripper.TeleopPeriodic();

		{
			float speed = 1.0f;

			if (flightStickLeft.getRawButtonPressed(1)) {
				setPointArm = setPointArm + 0.1;
			} else if (flightStickRight.getRawButtonPressed(1)) {
				setPointArm = setPointArm - 0.1;
			}
			arm.SetPoint(setPointArm);
			arm.Update(operator.getRawAxis(1));
			arm.debug();
			if (operator.getRawButton(2)) {
				arm.PowerManual(0);
			}
			driveTrain.SetCoast();
			ControllerDrive();
			driveTrain.Update();
		}

		// System.out.println(frontLeft.getSelectedSensorVelocity() + " front left " +
		// backLeft.getSelectedSensorVelocity()
		// + " back left " + frontRight.getSelectedSensorVelocity() + " front right "
		// + backRight.getSelectedSensorVelocity() + " back right");

		pwrm = 12 / (float) RobotController.getBatteryVoltage();
		float rampTime = 0.0f;
		// frontLeft.configOpenloopRamp(rampTime);
		// backLeft.configOpenloopRamp(rampTime);
		// frontRight.configOpenloopRamp(rampTime);
		// backRight.configOpenloopRamp(rampTime);

		driveTrain.SendData();
		SmartDashboard.putBoolean("RobotEnabled", true);

		// Lime Light
		if (flightStickLeft.getRawButton(2)) {
			limelight.Position(driveTrain);
			driveTrain.SetBreak();
		} else {
			driveTrain.SetCoast();

			ControllerDrive();
		}

		UpdateMotors();
	}

	public void testInit() {

		// navx.reset();
		// climber.coastMode();

		// Controllers
		// driver = new Joystick(0);
		operator = new Joystick(1);
		flightStickLeft = new Joystick(3);
		flightStickRight = new Joystick(2);
	}

	public float Lerp(float v0, float v1, float t) {

		if (t < 0) {
			t = 0;

		} else if (t > 1) {
			t = 1;
		}

		return (v0 + t * (v1 - v0));
	}

	// ffloat navxTarget;
	// NavxTurnPID testTurn = new NavxTurnPID(driveTrain, navx, 10, 2.5f, navxPID);

	// DigitalInput beamTest = new DigitalInput(1);
	// beamTest.get();
	// intakeSeven.set(ControlMode.PercentOutput, 0.5f);
	// flightStickLeft.getRawButtonPressed(0);

	/*
	 * - run intake when flight stick top button pressed
	 * - unless the ball is in beam break then don't run the intake
	 * - run intake backwards always, regardless of ball when trigger button is
	 * pressed
	 */
	// DigitalInput beamTest = new DigitalInput(1);

	public void testPeriodic() {

		arm.PowerManual(0);
		arm.ResetEncoder();

		/*
		 * Ultrasonic.setAutomaticMode(true);
		 * double ultraInches = sensorUltrasonic.getRangeInches();
		 * System.out.println(ultraInches);
		 * float error = (float) (ultraInches - 50);
		 * float p = 0.03f;
		 * 
		 * float power = (float) -(error * p);
		 * 
		 * if (Math.abs(power) > 0.3f) {
		 * power = (power/Math.abs(power))*0.3f;
		 * }
		 * System.out.println(power);
		 * driveTrain.SetBothSpeed(power);
		 */

		/*
		 * if(beamTest.get()){
		 * if (flightStickLeft.getRawButton(2)) {
		 * intakeSeven.set(ControlMode.PercentOutput, 0.5f);
		 * } else if(flightStickLeft.getRawButton(1)) {
		 * intakeSeven.set(ControlMode.PercentOutput, -0.5f);
		 * } else {
		 * intakeSeven.set(ControlMode.PercentOutput, 0f);
		 * }
		 * 
		 * }
		 * else {
		 * intakeSeven.set(ControlMode.PercentOutput, 0f);
		 * }
		 */

		/*
		 * frontLeft.getSelectedSensorPosition();
		 * 
		 * 
		 * preShooterFive.set(ControlMode.PercentOutput, (preShooterFeed.calculate(15,
		 * 100)/RobotController.getBatteryVoltage()));
		 * 
		 * 
		 * // System.out.println(preShooterFive.getSelectedSensorVelocity()/2048*8.19 +
		 * "Calc");
		 * // System.out.println(preShooterFive.getSelectedSensorVelocity()/2048*600 +
		 * "RPM");
		 * // preShooterFive.set(ControlMode.PercentOutput, 0);
		 * SmartDashboard.putNumber("Shooter Rot Target", 0);
		 * 
		 * shooter.PowerManual(0);
		 * System.out.println(frontLeft.getSelectedSensorPosition());
		 * 
		 * 
		 * driveTrain.SetBothSpeed(0);
		 * 
		 * driveTrain.SetCoast();
		 * climber.coastMode();
		 * 
		 * // forTesting.set(ControlMode.PercentOutput, 0.5f);
		 * // limelight.SetLight(true);
		 * 
		 * if (flightStickLeft.getRawButtonPressed(0)) {
		 * // navxTarget
		 * navx.reset();
		 * testTurn = new NavxTurnPID(driveTrain, navx, 10, 2.5f, navxPID);
		 * }
		 * 
		 * // testTurn.Update();
		 */
		// System.out.println(frontLeft.getSelectedSensorPosition());
		driveTrain.SetBothSpeed(0);
		driveTrain.SetCoast();
		driveTrain.Update();
		UpdateMotors();
	}

	public void ControllerDrive() {
		if (arcadeDrive) {

			// Arcade
			// float horJoystick = TranslateController((float) driver.getRawAxis(2));
			// float verJoystick = TranslateController((float) driver.getRawAxis(1));

			// float horJoystick = DriveScaleSelector((float) driver.getRawAxis(2),
			// DriveScale.parabala);
			// float verJoystick = DriveScaleSelector((float) driver.getRawAxis(1),
			// DriveScale.parabala);

			// driveTrain.SetRightSpeed(-verJoystick + -horJoystick);
			// driveTrain.SetLeftSpeed(-verJoystick + horJoystick);
			// driveTrain.SetCoast();
		} else {
			// tank
			// float leftJoystick = DriveScaleSelector((float)
			// flightStickLeft.getRawAxis(1), DriveScale.parabala);
			// float rightJoystick = (DriveScaleSelector((float)
			// flightStickRight.getRawAxis(1), DriveScale.parabala));
			// driveTrain.SetRightSpeed((-rightJoystick));
			// driveTrain.SetLeftSpeed((-leftJoystick));

			float leftJoystick = DriveScaleSelector((float) -flightStickLeft.getRawAxis(1), DriveScale.linear);
			float rightJoystick = (DriveScaleSelector((float) flightStickRight.getRawAxis(0), DriveScale.linear));

			driveTrain.SetRightSpeed(-leftJoystick + -rightJoystick);
			driveTrain.SetLeftSpeed(-leftJoystick + rightJoystick);
			driveTrain.Update();

			// driveTrain.SetRightSpeed(leftJoystick);
			// driveTrain.SetLeftSpeed(rightJoystick);
			// driveTrain.SetCoast();

		}
	}

	public void UpdateMotors() {
		driveTrain.Update();
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