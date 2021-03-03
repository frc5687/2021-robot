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

    private double _angle;

    public Hood(OutliersContainer container) {
        super(container);
        try {
            _hood =
                    new CANSparkMax(
                            RobotMap.CAN.SPARKMAX.HOOD, CANSparkMaxLowLevel.MotorType.kBrushless);
            _hood.setInverted(INVERTED);

            _hoodEncoder = _hood.getEncoder();
            _hoodController = _hood.getPIDController();

            _hoodEncoder.setPositionConversionFactor(GEAR_RATIO * DISTANCE_PER_ROTATION);
            _hoodEncoder.setVelocityConversionFactor(GEAR_RATIO);

            _hoodController.setP(kP);
            _hoodController.setI(kI);
            _hoodController.setD(kD);
            _hoodController.setIZone(kIz);
            _hoodController.setFF(kFF);
            _hoodController.setOutputRange(MIN_OUTPUT, MAX_OUTPUT);

            _hoodController.setSmartMotionMaxVelocity(MAX_VEL, 0);
            _hoodController.setSmartMotionMinOutputVelocity(MIN_VEL, 0);
            _hoodController.setSmartMotionMaxAccel(MAX_ACCEL, 0);
            _hoodController.setSmartMotionAllowedClosedLoopError(TOLERANCE, 0);

            _hallEffect = new HallEffect(RobotMap.DIO.HOOD_HALL);

        } catch (Exception e) {
            error(e.getMessage());
        }
    }

    @Override
    public void periodic() {
        //        if(isHallTriggered()) {
        //            if (_hood.get() < 0) {
        //                setSpeed(0);
        //            }
        //            _angle = MIN_ANGLE;
        //            _hoodEncoder.setPosition(_angle * POSITION_TO_ANGLE); //TODO: Find Conversion
        //        }

    }

    @Override
    public void updateDashboard() {}

    /** @return Position of hood in meters. */
    public double getPosition() {
        return _hoodEncoder.getPosition();
    }

    public double getAngle() {
        _angle =
                getPosition()
                        * POSITION_TO_ANGLE; // TODO: find out the conversion of position to angle.
        return _angle;
    }

    /** @return Angular velocity in RPM. */
    public double getVelocity() {
        return _hoodEncoder.getVelocity();
    }

    /**
     * Uses SparkMax SmartMotion to control the angle of the hood.
     *
     * @param rads reference angle in radians.
     */
    public void setHoodAngle(double rads) {
        _hoodController.setReference(rads, ControlType.kSmartMotion);
    }

    public void setSpeed(double pow) {
        _hood.set(pow);
    }

    public boolean isHallTriggered() {
        return _hallEffect.get();
    }
}
