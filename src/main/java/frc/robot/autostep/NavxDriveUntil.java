package frc.robot.autostep;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.DriveTrain;

public class NavxDriveUntil extends AutoStep {

    public AHRS navx;
    public float degree;
    public float speed;
    public DriveTrain drivetrain;

    public NavxDriveUntil(AHRS navx, float degree, float speed, DriveTrain drivetrain) {
        this.navx = navx;
        this.degree = degree;
        this.speed = speed;
        this.drivetrain = drivetrain;
    }

    public void Begin() {

    }

    public void Update() {
        System.out.println(navx.getRoll());
        if (degree > navx.getRoll()) {
            drivetrain.SetBothSpeed(speed);
        } else {
            drivetrain.SetBothSpeed(0);
            isDone = true;
        }
    }
}
