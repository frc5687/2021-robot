/* (C)2021 */
package org.frc5687.infiniterecharge.robot.subsystems;

import static org.frc5687.infiniterecharge.robot.Constants.Hood.*;

import com.revrobotics.*;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.util.HallEffect;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Hood extends OutliersSubsystem {

    private CANSparkMax _hood;
    private CANEncoder _hoodEncoder;
    private CANPIDController _hoodController;

    private HallEffect _hallEffect;
    private HallEffect _hallEffectTop;

    private double _angle;

    public Hood(OutliersContainer container) {
        super(container);
        _hood =
                new CANSparkMax(
                        RobotMap.CAN.SPARKMAX.HOOD, CANSparkMaxLowLevel.MotorType.kBrushless);
        _hood.restoreFactoryDefaults();
        _hood.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _hood.setInverted(INVERTED);

        _hoodEncoder = _hood.getAlternateEncoder(8192);
        _hoodController = _hood.getPIDController();
        _hoodController.setFeedbackDevice(_hoodEncoder);

        _hoodController.setP(kP);
        _hoodController.setI(kI);
        _hoodController.setD(kD);
        _hoodController.setFF(kFF);
        _hoodController.setIZone(kIz);

        _hoodController.setSmartMotionMaxVelocity(MAX_VEL, 0);
        _hoodController.setSmartMotionMaxAccel(MAX_ACCEL, 0);

        _hallEffect = new HallEffect(RobotMap.DIO.HOOD_HALL);
        _hallEffectTop = new HallEffect(RobotMap.DIO.HOOD_HALL_TOP);
    }

    @Override
    public void periodic() {}

    @Override
    public void updateDashboard() {}

    public double getPosition() {
        return _hoodEncoder.getPosition();
    }

    public void setEncoderAngle(double angle) {
        _angle = angle;
        _hoodEncoder.setPosition(_angle / POSITION_TO_ANGLE);
    }

    public double getAngle() {
        _angle =
                getPosition()
                        * POSITION_TO_ANGLE; // TODO: find out the conversion of position to angle.
        return _angle;
    }

    public double getVelocity() {
        return _hoodEncoder.getVelocity();
    }

    public void setHoodAngle(double deg) {
        _hoodController.setReference(deg / POSITION_TO_ANGLE, ControlType.kSmartMotion);
    }

    public void setSpeed(double pow) {
        _hood.set(pow);
    }

    public boolean isHallTriggered() {
        return _hallEffect.get();
    }

    public boolean isTopHallTriggered() {
        return _hallEffectTop.get();
    }

    public double getOutput() {
        return _hood.getAppliedOutput();
    }
}
