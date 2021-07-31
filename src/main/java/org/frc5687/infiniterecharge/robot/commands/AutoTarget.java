/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import org.frc5687.infiniterecharge.robot.subsystems.Hood;
import org.frc5687.infiniterecharge.robot.subsystems.Shooter;

public class AutoTarget extends OutliersCommand {

    private DriveTrain _drivetrain;
    private Shooter _shooter;
    private Hood _hood;
    private OI _oi;
    private double _angle;
    private double _rpm;
    private boolean _override;

    public AutoTarget(
            DriveTrain drivetrain,
            Shooter shooter,
            Hood hood,
            OI oi,
            double angle,
            double rpm,
            boolean override) {
        _drivetrain = drivetrain;
        _shooter = shooter;
        _hood = hood;
        _oi = oi;
        _rpm = rpm;
        _angle = angle;
        _override = override;
        addRequirements(_shooter, _hood);
    }

    @Override
    public void initialize() {
        _drivetrain.setUseAutoAim(true);
    }

    @Override
    public void execute() {
        super.execute();
        if (_override) {
            _hood.setPosition(_angle);
            _shooter.setVelocitySpeed(_rpm);
        } else {
            _hood.setPosition(63);
            _shooter.setVelocitySpeed(4500);
        }
    }

    @Override
    public boolean isFinished() {
        return _oi.isKillAllPressed();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _drivetrain.setUseAutoAim(false);
        _hood.setPosition(Constants.Hood.MIN_ANGLE);
        Command hoodCommand = _hood.getDefaultCommand();
        if (hoodCommand instanceof IdleHood) {
            ((IdleHood) hoodCommand).setZeroing(true);
        }
    }
}
