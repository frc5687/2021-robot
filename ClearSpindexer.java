package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.Spindexer;

public class ClearSpindexer extends OutliersCommand{

    private final Spindexer spindexer;

    public ClearSpindexer(Spindexer _spindexer){
        spindexer = _spindexer;
    }

    @Override
    public void initialize(){
        super.initialize();
    }

    @Override
    public void execute(){
        super.execute();
        spindexer.setSpindexerSpeed(0.25);
    }

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
    }
}
