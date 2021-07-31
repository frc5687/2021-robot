/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.subsystems.*;

public class KillAll extends OutliersCommand {
    private DriveTrain _driveTrain;
    private Shooter _shooter;
    private Spindexer _spindexer;
    private Hood _hood;
    private Climber _climber;
    private boolean _finished;

    public KillAll(
            DriveTrain driveTrain,
            Shooter shooter,
            Spindexer spindexer,
            Hood hood,
            Climber climber) {
        _driveTrain = driveTrain;
        _shooter = shooter;
        _hood = hood;
        _climber = climber;
    }

    @Override
    public void initialize() {
        error("Killing commands");
        super.initialize();
        _finished = true;
        _driveTrain.startModules();
        _shooter.setVelocitySpeed(0);
        _hood.setSpeed(0);
        _climber.setWinchSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return _finished;
    }

    @Override
    public void end(boolean interrupted) {
        error("Ending KillAll Command");
    }
}
