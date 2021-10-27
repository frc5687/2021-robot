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
        //Spark Max for the spindexer
        _spindexer = new CANSparkMax(RobotMap.CAN.SPARKMAX.SPINDEXER, CANSparkMaxLowLevel.MotorType.kBrushless);
        _spindexerEncoder = _spindexer.getEncoder();
        //Sparx Max for feeder
        _feeder = new CANSparkMax(RobotMap.CAN.SPARKMAX.FEEDER, CANSparkMaxLowLevel.MotorType.kBrushless);
        _feederEncoder = _feeder.getEncoder();
        //Invert spindexer and feeder
        _spindexer.setInverted(Constants.Spindexer.SPINDEXER_INVERTED);
        _feeder.setInverted(Constants.Spindexer.FEEDER_INVERTED);
        //Set coast modes
        _spindexer.setIdleMode(CANSparkMax.IdleMode.kCoast);
        _feeder.setIdleMode(CANSparkMax.IdleMode.kCoast);
        //Sets statuses for spindexer and feeder
        _spindexer.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        _spindexer.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 1000);
        _spindexer.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 1000);
        _feeder.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        _feeder.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 1000);
        _feeder.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 1000);
    }

    @Override
    public void periodic() {
        //        _feederEncoder.setPosition(0);
        //        metric("feeder", _feederEncoder.getPosition());
    }

    // Sets speed of motor -1 to 1.
    public void setSpindexerSpeed(double speed) {
        //Set spindexer feed
        _spindexer.set(speed);
    }

    public void setFeederSpeed(double speed) {
        //Set feeder speed
        _feeder.set(speed);
    }

    /**
     * Gets the Angular Velocity from the integrated encoder of the NEO/Baby NEO
     *
     * @return Angular Velocity in RPM
     */
    public double getSpindexerVelocity() {
        //The spindexers velocity
        return _spindexerEncoder.getVelocity();
    }

    public double getFeederVelocity() {
        //The feeders velocity
        return _feederEncoder.getVelocity();
    }

    // Not implemented yet, but yet it's been implemented...  
    public double getSpindexerAngle() {
        //Spindexers angle what ever that means
        return _spindexerEncoder.getPosition();
    }

    @Override
    public void updateDashboard() {}
}
