package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;

public class Gripper {

    public CANSparkMax motorGripper = new CANSparkMax(5, MotorType.kBrushless);

    private Robot robot;

    private boolean close = false;
    private boolean doClose = false;

    public Gripper(Robot robot) {
        this.robot = robot;
    }

    public void AutoPeriodic() {
        if (doClose) {
            motorGripper.set(-0.6f);
        }
    }

    public void teleopPeriodic() {

        if (robot.operator.getRawButtonPressed(7)) {
            close = true;
        }

        if (close) {
            motorGripper.set(-0.2);
        }

        if (robot.operator.getRawButton(8)) {
            close = false;
            motorGripper.set(0.2);
        } else if (close == false) {
            motorGripper.set(0.0);
        }
    }

    public void OpenGripper() {
        close = false;
    }

    public void AutoClose() {
        doClose = true;
    }

    public void AutoClear() {
        doClose = false;
    }
}
