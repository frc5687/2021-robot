/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands.climber;

import org.frc5687.infiniterecharge.robot.commands.OutliersCommand;
import org.frc5687.infiniterecharge.robot.subsystems.Climber;
import org.frc5687.infiniterecharge.robot.subsystems.Shooter;
import org.frc5687.infiniterecharge.robot.subsystems.Climber.Position;

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

    public Position climberPos(){
        return _climber.getArmPosition();
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
