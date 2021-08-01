/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands.climber;

import org.frc5687.infiniterecharge.robot.commands.OutliersCommand;
import org.frc5687.infiniterecharge.robot.subsystems.Climber;
import org.frc5687.infiniterecharge.robot.subsystems.Shooter;

public class RaiseArm extends OutliersCommand {
    private Climber _climber;
    private Shooter _shooter;

    public RaiseArm(Climber climber, Shooter shooter) {
        _climber = climber;
        _shooter = shooter;
        addRequirements(climber, _shooter);
    }

    @Override
    public void initialize() {
        super.initialize();
        _climber.raiseArm();
        _shooter.setVelocitySpeed(0);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
