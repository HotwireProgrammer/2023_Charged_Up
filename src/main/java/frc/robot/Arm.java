package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {

    // Arm
	public TalonSRX motorArm = new TalonSRX(53);
    public double armP = 0.4;
    public double armI = 0.00;
    public double armD = 0.00;

    public double encoderArmTicks = motorArm.getSelectedSensorPosition();
    public double encoderArmRadians = (encoderArmTicks / 35000 * 3.14);
    public double powerArm = 0;

    public PIDController pidArm = new PIDController(armP, armI, armD);

    public double idlePowerArm = 0.18;
    public double setPointArm = 0;

    public Arm() {

		motorArm.setNeutralMode(NeutralMode.Brake);
        SmartDashboard.putNumber("arm p", armP);
		SmartDashboard.putNumber("arm i", armI);
		SmartDashboard.putNumber("arm d", armD);
        armP = SmartDashboard.getNumber("arm p", armP);
        armI = SmartDashboard.getNumber("arm i", armI);
        armD = SmartDashboard.getNumber("arm d", armD);
        PIDController pidArm = new PIDController(armP, armI, armD);
    }
    
    public void Update(double VertStick){
     encoderArmTicks = motorArm.getSelectedSensorPosition();
     encoderArmRadians = (encoderArmTicks / 35000 * 3.14);
    System.out.println(pidArm.calculate(encoderArmRadians,setPointArm));
    //  powerArm = idlePowerArm * Math.cos(encoderArmRadians) + pidArm.calculate(encoderArmRadians,setPointArm);
    powerArm = idlePowerArm * Math.cos(encoderArmRadians) + VertStick;

     motorArm.set(ControlMode.PercentOutput,powerArm*10/RobotController.getBatteryVoltage());
     System.out.println(setPointArm);
    }

    public void debug(){
        System.out.println(setPointArm+" setpoint arm and arm pos "+encoderArmRadians/3.14+" and arm power out "+powerArm);
    }
     
    public void SetPoint(double setPointArm){
        this.setPointArm = setPointArm;
    }

    public void ResetEncoder(){
		motorArm.setSelectedSensorPosition(0);
    }
    public void PowerManual(double powerArm){
        motorArm.set(ControlMode.PercentOutput, powerArm);
    }

    // Arm Control


    // double powerArm =
    // (-flightStickLeft.getRawAxis(1)*10/RobotController.getBatteryVoltage());


}
