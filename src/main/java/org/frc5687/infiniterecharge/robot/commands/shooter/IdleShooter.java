/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands.shooter;

import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.commands.OutliersCommand;
import org.frc5687.infiniterecharge.robot.subsystems.Shooter;

public class IdleShooter extends OutliersCommand {

    private Shooter _shooter;
    private OI _oi;

    public IdleShooter(Shooter shooter, OI oi) {
        _shooter = shooter;
        _oi = oi;
        addRequirements(_shooter);
    }

    @Override
    public void initialize() {
        super.initialize();
        _shooter.setVelocitySpeed(_shooter.getReference());
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
