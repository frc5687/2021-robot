/* (C)5687-2021 */
package org.frc5687.infiniterecharge.robot.commands;

import static org.frc5687.infiniterecharge.robot.Constants.DriveTrain.*;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.controller.PIDController;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import org.frc5687.infiniterecharge.robot.util.Helpers;

public class Drive extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private final SlewRateLimiter _vxFilter;
    private final SlewRateLimiter _vyFilter;
    private final PIDController _visionController;
    private final OI _oi;

    public Drive(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _oi = oi;
        _vxFilter = new SlewRateLimiter(3.0); //Add to constent 
        _vyFilter = new SlewRateLimiter(3.0);
        _visionController = new PIDController(VISION_kP, VISION_kI, VISION_kD);
        error("Staring driving");
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        _driveTrain.startModules();
    }

    @Override
    public void execute() {
        super.execute();
        //  driveX and driveY are swapped due to coordinate system that WPILib uses.
//        double vx = Helpers.applySensitivityFactor(_vxFilter.calculate(-_oi.getDriveY()) * MAX_MPS, SENSITIVITY_VX);
        double vx = _vxFilter.calculate(Helpers.applySensitivityFactor(-_oi.getDriveY(),SENSITIVITY_VX)) * MAX_MPS;
//        double vy = Helpers.applySensitivityFactor(_vyFilter.calculate(_oi.getDriveX()) * MAX_MPS, SENSITIVITY_VY);
        double vy = _vyFilter.calculate(Helpers.applySensitivityFactor(_oi.getDriveX(), SENSITIVITY_VY)) * MAX_MPS;

        metric("aim", _driveTrain.autoAim());
        metric("hasTarget", _driveTrain.hasVisionTarget());
        double rot =
                (_driveTrain.autoAim() && _driveTrain.hasVisionTarget())
                        ? _visionController.calculate(_driveTrain.getLimelightYaw())
                        : _oi.getRotationX() * (_driveTrain.getManualAim() ? (MAX_ANG_VEL / 4.0) : MAX_ANG_VEL);

        _driveTrain.drive(vx, vy, rot, true);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
