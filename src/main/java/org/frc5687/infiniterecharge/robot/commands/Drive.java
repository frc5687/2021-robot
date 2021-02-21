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
        _oi = oi;
        _vxLimiter = new SlewRateLimiter(0.5);
        _vyLimiter = new SlewRateLimiter(0.5);
        _rotLimiter = new SlewRateLimiter(0.5);
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        // this is correct because of coordinate system.

        double vx =
                _vxLimiter.calculate(
                                Helpers.applySensitivityFactor(_oi.getDriveY(), SENSITIVITY_VX))
                        * MAX_MPS;
        double vy =
                _vyLimiter.calculate(
                                Helpers.applySensitivityFactor(-_oi.getDriveX(), SENSITIVITY_VY))
                        * MAX_MPS;
        metric("vx", vx);
        metric("vy", vy);
        double rot =
                _rotLimiter.calculate(
                                Helpers.applySensitivityFactor(
                                        -_oi.getRotationX(), SENSITIVITY_OMEGA))
                        * MAX_ANG_VEL;
        _driveTrain.drive(vx, vy, rot, true, _oi.holdAngle());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
