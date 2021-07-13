/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.Spindexer;

public class IdleSpindexer extends OutliersCommand {

    private Spindexer _spindexer;

    public IdleSpindexer(Spindexer spindexer) {
        _spindexer = spindexer;
        addRequirements(_spindexer);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        _spindexer.setSpindexerSpeed(Constants.Spindexer.SPINDEXER_IDLE_SPEED);
        _spindexer.setFeederSpeed(Constants.Spindexer.FEEDER_IDLE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}