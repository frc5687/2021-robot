/* (C)2021 */
package org.frc5687.infiniterecharge.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Spindexer extends OutliersSubsystem {
    private CANSparkMax _spindexer;
    private CANEncoder _spindexerEncoder;

    private CANSparkMax _feeder;
    private CANEncoder _feederEncoder;

    public Spindexer(OutliersContainer container) {
        super(container);
                try {
        _spindexer =
                new CANSparkMax(
                        RobotMap.CAN.SPARKMAX.SPINDEXER, CANSparkMaxLowLevel.MotorType.kBrushless);
        _spindexerEncoder = _spindexer.getEncoder();

        _feeder =
                new CANSparkMax(
                        RobotMap.CAN.SPARKMAX.FEEDER, CANSparkMaxLowLevel.MotorType.kBrushless);
        _feederEncoder = _feeder.getEncoder();

        _spindexer.setCANTimeout(500);
        _feeder.setCANTimeout(500);

        _spindexer.setInverted(Constants.Spindexer.SPINDEXER_INVERTED);
        _feeder.setInverted(Constants.Spindexer.FEEDER_INVERTED);

        _spindexer.setIdleMode(CANSparkMax.IdleMode.kCoast);
        _feeder.setIdleMode(CANSparkMax.IdleMode.kCoast);
        _spindexer.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20);
        _spindexer.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 200);
        _spindexer.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 200);
        _feeder.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20);
        _feeder.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 200);
        _feeder.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 200);
                } catch (Exception e) {
                    error(e.getMessage());
                }
    }

    @Override
    public void periodic() {
        //        _feederEncoder.setPosition(0);
        //        metric("feeder", _feederEncoder.getPosition());
    }

    // Sets speed of motor -1 to 1.
    public void setSpindexerSpeed(double speed) {
        _spindexer.set(speed);
    }

    public void setFeederSpeed(double speed) {
        _feeder.set(speed);
    }

    /**
     * Gets the Angular Velocity from the integrated encoder of the NEO/Baby NEO
     *
     * @return Angular Velocity in RPM
     */
    public double getSpindexerVelocity() {
        return _spindexerEncoder.getVelocity();
    }

    public double getFeederVelocity() {
        return _feederEncoder.getVelocity();
    }

    // Not implemented yet
    public double getSpindexerAngle() {
        return _spindexerEncoder.getPosition();
    }

    @Override
    public void updateDashboard() {}
}
