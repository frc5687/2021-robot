package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.Spindexer;

public class ClearSpindexer extends OutliersCommand {
    private Spindexer _spindexer;

    public ClearSpindexer(Spindexer spindexer) {
        _spindexer = spindexer;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        _spindexer.setSpindexerSpeed(Constants.Spindexer.SHIMMY);
        _spindexer.setFeederSpeed(Constants.Spindexer.SHIMMY);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
