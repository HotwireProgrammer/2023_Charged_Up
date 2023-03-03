package frc.robot;

import java.time.OffsetTime;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {

    // Arm
    public CANSparkMax motorArmRetraction = new CANSparkMax(13, MotorType.kBrushless);
    public CANSparkMax motorArm1 = new CANSparkMax(11, MotorType.kBrushless);
    public CANSparkMax motorArm2 = new CANSparkMax(12, MotorType.kBrushless);

    public RelativeEncoder encoderArmRevolutions = motorArm1.getEncoder();
    public RelativeEncoder encoderArmDistance = motorArmRetraction.getEncoder();

    public double armP = 0.4;
    public double armI = 0.00;
    public double armD = 0.00;

    public double voltsArm = 0;

    public boolean extended = false;

    private float offset = 0;
    // 61 distance
    public PIDController pidArm = new PIDController(armP, armI, armD);
    // 1.36
    public double idlePowerArm = 2.03;
    public double setPointArm = 0;

    public Arm() {

        SmartDashboard.putNumber("arm p", armP);
        SmartDashboard.putNumber("arm i", armI);
        SmartDashboard.putNumber("arm d", armD);
        armP = SmartDashboard.getNumber("arm p", armP);
        armI = SmartDashboard.getNumber("arm i", armI);
        armD = SmartDashboard.getNumber("arm d", armD);
        PIDController pidArm = new PIDController(armP, armI, armD);
    }

    public void OffsetGravity(boolean cone, boolean extended) {
        // if (cone) {
        // if (extended) {
        // idlePowerArm = 3;
        // } else {
        // idlePowerArm = 4;
        // }
        // } else {
        // if (extended) {
        // idlePowerArm = 2.25;
        // } else {
        // idlePowerArm = 2;

        // }
    }

    public void Update(double VertStick, Joystick operator) {
        idlePowerArm = Robot.Lerp(0.91f, 2.03f, (float) (encoderArmDistance.getPosition() / 61.0f));

        // System.out.println(encoderArmDistance.getPosition());

        // motorArmRetraction.set(VertStick);

        encoderArmRevolutions = motorArm1.getEncoder();
        float encoderArmRadians = (float) ((encoderArmRevolutions.getPosition() - offset) * 2.0f * 3.14f / 24.0f
                + 3.14f / 2.0f);

        // pidArm.calculate(encoderArmRadians,setPointArm);
        // voltsArm = 10 * VertStick;
        voltsArm = -idlePowerArm * Math.cos(encoderArmRadians) + 10 * VertStick;
        System.out.println(voltsArm+ "volts arm");
        if (operator.getRawButton(9)) {
            voltsArm = 7.5 * VertStick;
        }

        motorArm1.setVoltage(voltsArm);
        motorArm2.setVoltage(voltsArm);

        // System.out.println(voltsArm + " powerV");
        // System.out.println(encoderArmRadians + " radians");
        // System.out.println((encoderArmRevolutions.getPosition()-offset + 0.5) +
        // "math");
        // System.out.println(offset + "offset");

        // arm retract and extend
        if (operator.getPOV() == -1) {
            if (!extended) {
                motorArmRetraction.setVoltage(0.8f);
            } else {
                motorArmRetraction.set(0);
            }
        } else if (operator.getPOV() > 310 || operator.getPOV() < 50) {

            if (encoderArmDistance.getPosition() < 60) {
                motorArmRetraction.set(-0.6f);
            } else {
                motorArmRetraction.set(0.0f);
            }
            extended = true;
        } else if (130 < operator.getPOV() && operator.getPOV() < 230) {
            if (encoderArmDistance.getPosition() > 5) {
                motorArmRetraction.set(0.6f);
            } else {
                motorArmRetraction.set(0.2f);
            }
        }

        // System.out.println(setPointArm);
    }

    public void debug() {
        // System.out.println(setPointArm + " setpoint arm and arm pos " +
        // encoderArmRadians / 3.14 + " and arm power out " + powerArm);
    }

    public void SetPoint(double setPointArm) {
        this.setPointArm = setPointArm;
    }

    public void ResetEncoder() {
        encoderArmDistance.setPosition(0.0f);

        offset = (float) encoderArmRevolutions.getPosition();

    }

    public void PowerManual(double voltsArm) {
        // motorArm.set(ControlMode.PercentOutput, powerArm);
    }

    public void Extend(double power) {
        motorArmRetraction.set(power);
    }

    // Arm Control

    // double powerArm =
    // (-flightStickLeft.getRawAxis(1)*10/RobotController.getBatteryVoltage());

}
