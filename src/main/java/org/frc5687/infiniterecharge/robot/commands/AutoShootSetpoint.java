package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.Hood;
import org.frc5687.infiniterecharge.robot.subsystems.Shooter;
import org.frc5687.infiniterecharge.robot.subsystems.Spindexer;

public class AutoShootSetpoint extends Shoot {

    private final Shooter _shooter;
    private final Hood _hood;

    private final double _setpointShooter;
    private final double _setpointHood;
    private Long _endTime;

    public AutoShootSetpoint(Shooter shooter, Spindexer spindexer, Hood hood, double setpointRPM, double setpointDeg) {
        super(shooter, spindexer);
        _shooter = shooter;
        _hood = hood;
        _setpointShooter = setpointRPM;
        _setpointHood = setpointDeg;
        addRequirements(_shooter, _hood);
    }

    @Override
    public void initialize() {
        super.initialize();
        _endTime = null;
        _shooter.setVelocitySpeed(_setpointShooter);
        _hood.setHoodAngle(_setpointHood);
    }

    @Override
    public void execute() {
        super.execute();
        if (_endTime == null) {
            _endTime = System.currentTimeMillis() + Constants.Shooter.TIMEOUT;
        }
    }

    @Override
    public boolean isFinished() {
        if (_endTime == null) {
            return false;
        }
        return System.currentTimeMillis() >= _endTime;
    }
}
