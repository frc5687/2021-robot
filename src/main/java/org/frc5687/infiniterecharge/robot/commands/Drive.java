/* (C)5687-2021 */
package org.frc5687.infiniterecharge.robot.commands;

import static org.frc5687.infiniterecharge.robot.Constants.DriveTrain.MAX_ANG_VEL;
import static org.frc5687.infiniterecharge.robot.Constants.DriveTrain.MAX_MPS;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;

public class Drive extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private final SlewRateLimiter _vxFilter;
    private final SlewRateLimiter _vyFilter;

    private final OI _oi;

    public Drive(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _oi = oi;
        _vxFilter = new SlewRateLimiter(3.0);
        _vyFilter = new SlewRateLimiter(3.0);
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
        double vx = _vxFilter.calculate(-_oi.getDriveY()) * MAX_MPS;
        double vy = _vyFilter.calculate(_oi.getDriveX()) * MAX_MPS;
        double rot = -_oi.getRotationX() * MAX_ANG_VEL;

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
