
package frc.robot;

import java.time.OffsetTime;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsytem {

    // public SwerveModule(int driveMotorId, int turningMotorId, boolean
    // driveMotorReversed,
    // boolean turningMotorReversed, double absoluteEncoderOffsetNative)

    // private final SwerveModule frontLeft = new SwerveModule(31, 41, false, false, 955.0);
    // private final SwerveModule frontRight = new SwerveModule(32, 42, false, false, 1083.0);
    // private final SwerveModule backLeft = new SwerveModule(33, 43, false, false, 1283.0);
    // private final SwerveModule backRight = new SwerveModule(34, 44, false, false, 955.0);

    private final SwerveModule frontLeft = new SwerveModule(31, 41, false, false, 0.0);
    private final SwerveModule frontRight = new SwerveModule(32, 42, false, false, 0.0);
    private final SwerveModule backLeft = new SwerveModule(33, 43, false, false, 0.0);
    private final SwerveModule backRight = new SwerveModule(34, 44, false, false, 0.0);
    private AHRS navx;
    // private final SwerveModule SPARECHANGENAME = new SwerveModule(35, 45, false,
    // false, 0.0);

    public double getHeading(AHRS navx) {
        this.navx = navx;
        return Math.IEEEremainder(navx.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(navx.getAngle());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 5.0);

        frontLeft.setDesiredState(desiredStates[1]);
        frontRight.setDesiredState(desiredStates[0]);
        backLeft.setDesiredState(desiredStates[3]);
        backRight.setDesiredState(desiredStates[2]);
    }

}
