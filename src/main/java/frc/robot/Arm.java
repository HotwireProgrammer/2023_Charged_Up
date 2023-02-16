package frc.robot;

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

    public double armP = 0.4;
    public double armI = 0.00;
    public double armD = 0.00;

    public double voltsArm = 0;

    public float offset = 0;

    public PIDController pidArm = new PIDController(armP, armI, armD);

    public double idlePowerArm = 2.25;
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
        if (cone) {
            if (extended) {
                idlePowerArm = 3;
            } else {
                idlePowerArm = 4;
            }
        } else {
            if (extended) {
                idlePowerArm = 2.25;
            } else {
                idlePowerArm = 2;
            }
        }
    }

    public void Update(double VertStick, Joystick operator) {

        // motorArmRetraction.set(VertStick);
        RelativeEncoder encoderArmRevolutions = motorArm1.getEncoder();
        float encoderArmRadians = (float) (encoderArmRevolutions.getPosition() * Math.PI / 12) - offset;

        // pidArm.calculate(encoderArmRadians,setPointArm);
        voltsArm = 10 * VertStick;
        // voltsArm = idlePowerArm * Math.cos(encoderArmRadians) + VertStick;

        motorArm1.setVoltage(voltsArm);
        motorArm2.setVoltage(voltsArm);
        System.out.println(voltsArm + " powerV");

        //arm retract and extend
        if (operator.getRawButton(4)) {
            motorArmRetraction.set(0.3);
        } else if (operator.getRawButton(1)) {
            motorArmRetraction.set(-0.3);
        } else {
            motorArmRetraction.set(0.05);
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
        // motorArm.setSelectedSensorPosition(0);
    }

    public void PowerManual(double voltsArm) {
        // motorArm.set(ControlMode.PercentOutput, powerArm);
    }

    // Arm Control

    // double powerArm =
    // (-flightStickLeft.getRawAxis(1)*10/RobotController.getBatteryVoltage());

}
