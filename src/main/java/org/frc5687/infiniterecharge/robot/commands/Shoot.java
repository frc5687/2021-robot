/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.Shooter;
import org.frc5687.infiniterecharge.robot.subsystems.Spindexer;

public class Shoot extends OutliersCommand {

    private final Shooter _shooter;
    private final Spindexer _spindexer;

    private Long _endTime;

    public Shoot(Shooter shooter, Spindexer spindexer) {
        _shooter = shooter;
        _spindexer = spindexer;
        addRequirements(_spindexer);
    }

    @Override
    public void initialize() {
        super.initialize();
        _endTime = null;
    }

    @Override
    public void execute() {
        super.execute();
        if (_shooter.isAtTargetVelocity()) {
            _endTime = System.currentTimeMillis() + Constants.Shooter.TIMEOUT;
            _spindexer.setSpindexerSpeed(Constants.Spindexer.SPINDEXER_SPEED);
            _spindexer.setFeederSpeed(Constants.Spindexer.FEEDER_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        if (_endTime == null) {
            return false;
        }
        return System.currentTimeMillis() >= _endTime;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}