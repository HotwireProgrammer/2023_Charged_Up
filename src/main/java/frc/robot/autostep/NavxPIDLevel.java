package frc.robot.autostep;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.DriveTrain;
import frc.robot.HotPID;

public class NavxPIDLevel extends AutoStep {

    public AHRS navx;
    public float degree;
    public float speed;
    public DriveTrain drivetrain;
    public boolean didClimb;
    public HotPID navxPID = new HotPID("navx PID", 0.02f, 0.0f, 0.0f);

    public NavxPIDLevel(AHRS navx, DriveTrain drivetrain) {
        this.navx = navx;
        this.drivetrain = drivetrain;
    }

    public void Begin() {

    }

    public void Update() {
        navxPID.setpoint = 0.0f;
        System.out.println(navx.getPitch());
        speed = (float)navxPID.Calculate(navx.getPitch());
        if (Math.abs(speed) > 0.2f){
            speed = 0.2f*speed/Math.abs(speed);
        }

        if(Math.abs(navx.getPitch())<2.5f){
            speed = 0.0f;
        }
        drivetrain.SetBothSpeed(-speed);

        // if (navx.getPitch() > 10) {
        //     didClimb = true;
        // }
        // if (didClimb && navx.getPitch() < 3) {
        //     isDone = true;
        //     drivetrain.SetBothSpeed(0);

        // }
      
    }
}
