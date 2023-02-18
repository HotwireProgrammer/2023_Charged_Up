package frc.robot.autostep;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DriveTrain;

public class NavxDriveUntil extends AutoStep {

    public AHRS navx;
    public float degree;
    public float speed;
    public DriveTrain drivetrain;
    public boolean didClimb;
    private double p = 0.05;
    private double i = 0;
    private double d = 0;
    public PIDController navxPID = new PIDController(p, i, d);

    public NavxDriveUntil(AHRS navx, float degree, float speed, DriveTrain drivetrain) {
        this.navx = navx;
        this.degree = degree;
        this.speed = speed;
        this.drivetrain = drivetrain;
        SmartDashboard.putNumber("navx p", p);
        SmartDashboard.putNumber("navx i", i);
        SmartDashboard.putNumber("navx d", d);
        p = SmartDashboard.getNumber("arm p", p);
        i = SmartDashboard.getNumber("arm i", i);
        d = SmartDashboard.getNumber("arm d", d);
        PIDController navxPID = new PIDController(p, i, d);

    }

    public void Begin() {

    }

    public void Update() {
        System.out.println(navx.getRoll());
        speed = (float) navxPID.calculate(navx.getRoll(), 0);
        if (speed > 0.5f) {
            speed = 0.5f;
        }

        drivetrain.SetBothSpeed(speed);

        if (navx.getRoll() < -10) {
            didClimb = true;
        }
        if (didClimb && navx.getRoll() > -3) {
            isDone = true;
            drivetrain.SetBothSpeed(0);

        }

    }
}
