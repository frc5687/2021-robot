package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.subsystems.Shooter;

public class IdleShooter extends OutliersCommand {

    private Shooter _shooter;

    public IdleShooter(Shooter shooter) {
        _shooter = shooter;
        addRequirements(_shooter);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
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
