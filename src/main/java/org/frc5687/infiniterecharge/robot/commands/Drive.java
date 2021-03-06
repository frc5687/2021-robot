/* (C)2020-2021 */
package org.frc5687.infiniterecharge.robot.commands;

import static org.frc5687.infiniterecharge.robot.Constants.DriveTrain.*;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import org.frc5687.infiniterecharge.robot.util.Helpers;

public class Drive extends OutliersCommand {

    private DriveTrain _driveTrain;
    private SlewRateLimiter _vxLimiter;
    private SlewRateLimiter _vyLimiter;
    private SlewRateLimiter _rotLimiter;
    private OI _oi;

    public Drive(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _vxLimiter = new SlewRateLimiter(4);
        _vyLimiter = new SlewRateLimiter(4);
        _rotLimiter = new SlewRateLimiter(3);
        _oi = oi;
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        double vx =
                //                _vxLimiter.calculate(
                Helpers.applySensitivityFactor(_oi.getDriveY(), SENSITIVITY_VX) * MAX_MPS / 2;
        double vy =
                //                _vyLimiter.calculate(
                Helpers.applySensitivityFactor(-_oi.getDriveX(), SENSITIVITY_VY) * MAX_MPS / 2;
        // this is correct because of coordinate system.
        metric("vx", vx);
        metric("vy", vy);
        double rot =
                //                _rotLimiter.calculate(
                Helpers.applySensitivityFactor(-_oi.getRotationX(), SENSITIVITY_OMEGA)
                        * MAX_ANG_VEL
                        / 4;
        _driveTrain.drive(vx, vy, rot, true, _oi.holdAngle());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
