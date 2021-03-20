/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.Shooter;
import org.frc5687.infiniterecharge.robot.subsystems.Spindexer;

public class AutoShoot extends OutliersCommand {

    private Spindexer _spindexer;
    private Shooter _shooter;

    public AutoShoot(Spindexer spindexer, Shooter shooter) {
        _spindexer = spindexer;
        _shooter = shooter;
        addRequirements(_spindexer, _shooter);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        _spindexer.setSpindexerSpeed(Constants.Spindexer.SPINDEXER_SPEED);
        _spindexer.setFeederSpeed(Constants.Spindexer.FEEDER_SPEED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
