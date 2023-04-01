package frc.robot;

import java.time.OffsetTime;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.AHRSProtocol.TuningVar;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

    private final double p = 0.7;
    private final double i = 0.0;
    private final double d = 0.0;

    private final TalonSRX driveMotor;
    private final TalonSRX turningMotor;

    private final double absoluteEncoderOffsetNative;
    private final double RevsToRevs = 3.6;
    private final double NativeToRevs = 4096.0;
    private final double RevsToRadians = 3.14*2;


    private final PIDController turningPidController;
    public Object stop;

    // private final boolean absoluteEncoderReversed;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed,
            boolean turningMotorReversed, double absoluteEncoderOffsetNative) {

        this.absoluteEncoderOffsetNative = absoluteEncoderOffsetNative;

        driveMotor = new TalonSRX(driveMotorId);
        turningMotor = new TalonSRX(turningMotorId);

        turningMotor.configSelectedFeedbackCoefficient(1.0);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);
        

        // turningMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        turningMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        turningMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setNeutralMode(NeutralMode.Coast);


        // driveEncoder.setPositionConversionFactor(0.239f);
        // driveEncoder.setVelocityConversionFactor(0.293f);

        turningPidController = new PIDController(p, i, d);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition();
    }

    public double getTurningPosition() {
        return (turningMotor.getSelectedSensorPosition() - absoluteEncoderOffsetNative)*(RevsToRadians*1.0/RevsToRevs/NativeToRevs);
    }

    public double getDriveVelocity() {
        return 10.0 * driveMotor.getSelectedSensorVelocity();
    }

    public double getTurningVelocity() {
        return 10.0 * turningMotor.getSelectedSensorVelocity()*(RevsToRadians*1.0/RevsToRevs/NativeToRevs);
    }

    public double getAbsoluteEncoderRad() {
        
        // turningMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        return turningMotor.getSelectedSensorPosition()*(RevsToRadians*1.0/RevsToRevs/NativeToRevs);
}

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0.0);
        // double EncoderResetDouble = (getAbsoluteEncoderRad()) - absoluteEncoderOffsetNative;
        // turningMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        // turningMotor.setSelectedSensorPosition(EncoderResetDouble);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / 5.0);
        float pidVal = (float)(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

        


        if(pidVal < -0.99f){
            pidVal = -0.99f;
        }else if(pidVal > 0.99f){
            pidVal = 0.99f;
        }

        turningMotor.set(ControlMode.PercentOutput, pidVal);
        

            // System.out.println(getAbsoluteEncoderRad() + " abs encoder");
            // System.out.println(getTurningPosition() + " rel encoder");

                // System.out.println(getTurningPosition() + " get turning position");
                // System.out.println(state.angle.getRadians() + " state.angle.getRadians()");
                // System.out.println(pidVal + " pid");


        // SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "]
        // state", state.toString());
    }

    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0.0);
        turningMotor.set(ControlMode.PercentOutput, 0.0);
    }
}
