package frc.robot.autostep;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.DriveTrain;
import frc.robot.Arm;

public class ArmMove extends AutoStep {

    public Timer armTimer;
    public float time;
    public float speed;
    public Arm arm;

    public ArmMove(Arm arm, float time, float speed) {
        super();
        this.time = time;
        this.speed = speed;
        this.arm = arm;
        armTimer = new Timer();
    }

    public void Begin() {
        armTimer.reset();
        armTimer.start();
    }

    public void Update() {
        arm.Update(speed);
        if (armTimer.get() > time) {
            isDone = true;
            arm.Update(0);
        }
    }
}