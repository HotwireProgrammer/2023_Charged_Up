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

    public double armP = 1.0f;
    public double armI = 0.00;
    public double armD = 0.00;

    public boolean powerBool = false;

    public double voltsArm = 0;

    public boolean extended = false;

    private float offset = 0;
    // 61 distance
    public PIDController pidArm = new PIDController(armP, armI, armD);

    // 1.36
    public double idlePowerArm = 2.03;
    public double setPointArm = 3.14f / 2.0f;

    public boolean autoDoExtend = false;
    public boolean autoExtend = false;

    public int extendEncoderOut = -45;
    public int extendEncoderIn = -5;

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

        setPointArm = setPointArm + VertStick / 100.0f;
        if (setPointArm < 3.14f / 4.0f) {
            setPointArm = 3.14f / 4.0f;
        } else if (setPointArm > 3.14f * 3.0f / 4.0f) {
            setPointArm = 3.14f * 3.0f / 4.0f;
        }

        idlePowerArm = Robot.Lerp(1.0f, 3.0f, (float) (Math.abs(encoderArmDistance.getPosition()) / 77.0f));

        // motorArmRetraction.set(VertStick);

        encoderArmRevolutions = motorArm1.getEncoder();
        float encoderArmRadians = (float) ((encoderArmRevolutions.getPosition() - offset) * 2.0f * 3.14f / 24.0f
                + 3.14f / 2.0f);

        // voltsArm = 10 * VertStick;

         voltsArm = -idlePowerArm * Math.cos(encoderArmRadians) + 5 * VertStick;

        // System.out.println(-pidArm.calculate(encoderArmRadians, setPointArm) + " pid");
        // System.out.println(setPointArm/3.14f + " setpoint");

        //voltsArm = -idlePowerArm * Math.cos(encoderArmRadians) - pidArm.calculate(encoderArmRadians, setPointArm/3.14f);

        if (operator.getRawButton(9)) {
            // voltsArm = 7.5 * VertStick;
        }
        if(powerBool){
            voltsArm = 0.0f;
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

            if (encoderArmDistance.getPosition() > extendEncoderOut) {
                motorArmRetraction.set(-0.6f);
            } else {
                motorArmRetraction.set(0.0f);
            }
            extended = true;
        } else if (130 < operator.getPOV() && operator.getPOV() < 230) {
            if (encoderArmDistance.getPosition() < extendEncoderIn) {
                motorArmRetraction.set(0.6f);
            } else {
                motorArmRetraction.set(0.2f);
            }
        }

        // System.out.println(setPointArm);
    }

    public void AutoUpdate() {


        System.out.println(encoderArmDistance.getPosition()+" pos");

        if (!autoExtend) { return; }

        if (autoDoExtend) {
            if (encoderArmDistance.getPosition() > extendEncoderOut) {
                System.out.println("working");
                motorArmRetraction.set(-0.6f);
            } else {
                motorArmRetraction.set(0f);
            }

        } else {
            if (encoderArmDistance.getPosition() < extendEncoderIn) {
                System.out.println("working2");
                motorArmRetraction.set(0.6f);
            } else {

                motorArmRetraction.set(0.2f);
            }
        }

    }

    public void debug() {
        // System.out.println(setPointArm + " setpoint arm and arm pos " +
        // encoderArmRadians / 3.14 + " and arm power out " + powerArm);
    }

    // public void SetPoint(double setPointArm) {
    // this.setPointArm = setPointArm;
    // }

    public void ResetEncoder() {
        encoderArmDistance.setPosition(0.0f);
        offset = (float) encoderArmRevolutions.getPosition();
    }

    public void RetractManual(double speed) {
        motorArmRetraction.set(speed);
    }

    public void PowerManual(double voltsArm) {
        motorArm1.setVoltage(voltsArm);
        motorArm2.setVoltage(voltsArm);
    }

    public void Extend(boolean doExtend) {
        autoDoExtend = doExtend;
    }

    // Arm Control

    // double powerArm =
    // (-flightStickLeft.getRawAxis(1)*10/RobotController.getBatteryVoltage());

}
