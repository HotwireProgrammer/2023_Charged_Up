package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

public class Gripper {

    public CANSparkMax motorGripper = new CANSparkMax(14, MotorType.kBrushless);
    public CANSparkMax motorRotationGripper = new CANSparkMax(15, MotorType.kBrushless);

    private Robot robot;

    private Timer timer = new Timer();
    private boolean flipped = false;
    private boolean close = false;
    private double closingForce = 0;
    private boolean doClose = true;

    public Gripper(Robot robot) {
        this.robot = robot;
    }

    public void AutoPeriodic() {
        if (doClose) {
            motorGripper.set(0.6f);
        }
    }

    public void teleopPeriodic() {
        timer.start();

        // motorRotationGripper.set(robot.operator.getRawAxis(0) / 2.0f);
        float flipSpeed = 0.2f;
        if (robot.operator.getRawButtonPressed(3)) {
            if (flipped) {
                motorRotationGripper.set(flipSpeed);
                flipped = false;
            } else {
                flipped = true;
                motorRotationGripper.set(-flipSpeed);

            }
        }

        if (robot.operator.getRawButtonPressed(5)) {
            close = true;
            closingForce = 0.3;

        }
        if (robot.operator.getRawButtonPressed(7)) {
            close = true;
            closingForce = 0.5;
            robot.cone = true;
        }

        if (close) {
            motorGripper.set(closingForce);
        }

        if (robot.operator.getRawButton(2) || robot.flightStickLeft.getRawButton(1)) {
            timer.reset();
            robot.cone = false;
            close = false;
            motorGripper.set(-0.2);
        } else if (timer.get() > 1 && close == false) {
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
