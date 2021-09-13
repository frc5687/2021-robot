/* (C)2021 */
package org.frc5687.infiniterecharge.robot.subsystems;

import static org.frc5687.infiniterecharge.robot.Constants.Shooter.*;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.util.Helpers;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Shooter extends OutliersSubsystem {

    private TalonFX _leftShooter;
    private TalonFX _rightShooter;

    private double _rpm;

    public Shooter(OutliersContainer container) {
        super(container);
        _leftShooter = new TalonFX(RobotMap.CAN.TALONFX.LEFT_SHOOTER);
        _rightShooter = new TalonFX(RobotMap.CAN.TALONFX.RIGHT_SHOOTER);

        _leftShooter.follow(_rightShooter);

        _leftShooter.setInverted(LEFT_INVERTED);
        _rightShooter.setInverted(RIGHT_INVERTED);

        _rightShooter.config_kP(0, kP);
        _rightShooter.config_kI(0, kI);
        _rightShooter.config_kD(0, kD);
        _rightShooter.config_kF(0, kFF);
        _rightShooter.config_IntegralZone(0, kIz, 50);

        _rightShooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        _rightShooter.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        _leftShooter.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        _leftShooter.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

        _rightShooter.configClosedloopRamp(4);
        _rightShooter.selectProfileSlot(0, 0);
        _rpm = 0;
    }

    @Override
    public void periodic() {}

    public void setShooterSpeed(double speed) {
        metric("Speed", speed);
        _rightShooter.set(TalonFXControlMode.PercentOutput, speed);
    }

    public void setVelocitySpeed(double RPM) {
        _rpm = Helpers.limit(RPM, 0, MAX_RPM);
        _rightShooter.set(
                TalonFXControlMode.Velocity,
                (_rpm * Constants.Shooter.TICKS_TO_ROTATIONS / 600 / GEAR_RATIO));
    }

    public double getVelocity() {
        return _rightShooter.getSelectedSensorVelocity();
    }

    public double getReference() {
        return _rpm;
    }

    public void setReference(double rpm) {
        _rpm = Helpers.limit(rpm, 0, MAX_RPM);
    }

    public double getRPM() {
        return getVelocity()
                / Constants.Shooter.TICKS_TO_ROTATIONS
                * 600
                * Constants.Shooter.GEAR_RATIO;
    }

    @Override
    public void updateDashboard() {
        metric("Velocity/Ticks", getVelocity());
        metric("targetVel", isAtTargetVelocity());
        metric("Velocity/RPM", getRPM());
        metric("ref", getReference());
        metric("is At vel", isAtTargetVelocity());
    }

    public boolean isAtTargetVelocity() {
        return Math.abs(getReference() - getRPM()) < TOLERANCE;
    }

    public void setShooterFromReference() {
        setVelocitySpeed(_rpm);
    }
}
