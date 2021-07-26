/* (C)2021 */
package org.frc5687.infiniterecharge.robot.subsystems;

import static org.frc5687.infiniterecharge.robot.Constants.Hood.*;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.util.HallEffect;
import org.frc5687.infiniterecharge.robot.util.Helpers;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Hood extends OutliersSubsystem {

    private TalonSRX _hoodController;
    private HallEffect _hallEffect;
    private HallEffect _hallEffectTop;

    private double _reference;
    private double _position;

    public Hood(OutliersContainer container) {
        super(container);

        _hoodController = new TalonSRX(RobotMap.CAN.TALONSRX.HOOD);
        _hoodController.configFactoryDefault();
        _hoodController.setInverted(INVERTED);
        _hoodController.setNeutralMode(NeutralMode.Brake);
        _hoodController.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 200);
        _hoodController.setSensorPhase(SENSOR_PHASE_INVERTED);
        _hoodController.configMotionCruiseVelocity(CRUISE_VELOCITY);
        _hoodController.configMotionAcceleration(ACCELERATION);
        _hoodController.configVoltageMeasurementFilter(8);
        //        _hoodController.enableVoltageCompensation(true);
        _hoodController.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 20, 200);
        _hoodController.configClosedloopRamp(0, 200);
        _hoodController.config_kP(0, kP, 200);
        _hoodController.config_kI(0, kI, 200);
        _hoodController.config_kD(0, kD, 200);
        _hoodController.config_kF(0, kF, 200);
        _hoodController.config_IntegralZone(0, I_ZONE, 200);
        _hoodController.selectProfileSlot(0, 0);
        _hallEffect = new HallEffect(RobotMap.DIO.HOOD_HALL);
        _hallEffectTop = new HallEffect(RobotMap.DIO.HOOD_HALL_TOP);
        _hoodController.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        if (isBottomHallTriggered()) {
            _hoodController.setSelectedSensorPosition(MIN_ANGLE / TICKS_TO_DEGREES);
        }
        metric("Output", _hoodController.getMotorOutputPercent());
        //                if ((getMotorOutput() <= 0.0) && isBottomHallTriggered()) {
        //                    _hoodController.setSelectedSensorPosition(MIN_ANGLE /
        // TICKS_TO_DEGREES);
        //                    setSpeed(0);
        //            _position = Constants.Hood.MIN_ANGLE /
        //        }
        //        if ((getMotorOutput() >= 0.0) && isTopHallTriggered()) {
        //            setSpeed(0);
        //            _hoodController.setSelectedSensorPosition((int) _position);
        //            _position = Constants.Hood.MIN_ANGLE / Constants.Hood.TICKS_TO_DEGREES;
        //        }
    }

    @Override
    public void updateDashboard() {
        metric("test", getPositionDegrees());
    }

    public void setSpeed(double speed) {
        double limSpeed = Helpers.limit(speed, -MAX_SPEED, MAX_SPEED);
        _hoodController.set(ControlMode.PercentOutput, limSpeed);
    }

    public void setPosition(double angle) {
        _reference = Helpers.limit(angle, MIN_ANGLE, MAX_ANGLE);
        _hoodController.set(ControlMode.MotionMagic, _reference / Constants.Hood.TICKS_TO_DEGREES);
    }

    public double getPositionTicks() {
        return _hoodController.getSelectedSensorPosition(0);
    }

    public double getReference() {
        return _reference;
    }

    public double getMotorOutput() {
        return _hoodController.getMotorOutputPercent();
    }

    public double getPositionDegrees() {
        return getPositionTicks() * Constants.Hood.TICKS_TO_DEGREES;
    }

    public boolean isAtSetpoint() {
        return _hoodController.isMotionProfileFinished();
    }

    public void zeroSensors() {
        if (isBottomHallTriggered()) {
            _position = Constants.Hood.MIN_ANGLE / Constants.Hood.TICKS_TO_DEGREES;
            _reference = Constants.Hood.MIN_ANGLE;
        }
        _hoodController.setSelectedSensorPosition((int) _position);
    }

    public boolean isBottomHallTriggered() {
        return _hallEffect.get();
    }

    public boolean isTopHallTriggered() {
        return _hallEffectTop.get();
    }
}
