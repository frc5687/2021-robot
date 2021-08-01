/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands.shooter;

import org.frc5687.infiniterecharge.robot.commands.OutliersCommand;
import org.frc5687.infiniterecharge.robot.subsystems.Hood;
import org.frc5687.infiniterecharge.robot.subsystems.Shooter;

public class SetShooterSetpoint extends OutliersCommand {
    private Shooter _shooter;
    private Hood _hood;
    private double _hoodAngle;
    private double _shooterRPM;

    public SetShooterSetpoint(Shooter shooter, Hood hood, double angle, double rpm) {
        _shooter = shooter;
        _hood = hood;
        _hoodAngle = angle;
        _shooterRPM = rpm;
    }

    @Override
    public void initialize() {
        super.initialize();
        _hood.setReference(_hoodAngle);
        _shooter.setReference(_shooterRPM);
    }

    @Override
    public boolean isFinished() {
        return (_shooter.getReference() == _shooterRPM) && (_hood.getReference() == _hoodAngle);
    }

    @Override
    public void end(boolean interrupted) {}
}
