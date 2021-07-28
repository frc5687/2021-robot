package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import org.frc5687.infiniterecharge.robot.subsystems.Hood;
import org.frc5687.infiniterecharge.robot.subsystems.Shooter;

public class AutoTarget extends OutliersCommand {

    private DriveTrain _drivetrain;
    private Shooter _shooter;
    private Hood _hood;
    public AutoTarget(
            DriveTrain drivetrain,
            Shooter shooter,
            Hood hood) {
        _drivetrain = drivetrain;
        _shooter = shooter;
        _hood = hood;
        addRequirements(_shooter, _hood);
    }

    @Override
    public void initialize() {
        _drivetrain.setUseAutoAim(true);
    }

    @Override
    public void execute() {
        super.execute();
        _hood.setPosition(_hood.getHoodDesiredAngle(_drivetrain.getDistanceToTarget()));
        _shooter.setVelocitySpeed(_shooter.getDistanceSetpoint(_drivetrain.getDistanceToTarget()));
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _drivetrain.setUseAutoAim(false);
    }
}
