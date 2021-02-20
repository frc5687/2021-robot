/* (C)2020-2021 */
package org.frc5687.infiniterecharge.robot.commands;

import static org.frc5687.infiniterecharge.robot.Constants.DriveTrain.*;

import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import org.frc5687.infiniterecharge.robot.util.Helpers;

public class Drive extends OutliersCommand {

    private DriveTrain _driveTrain;
    private OI _oi;

    public Drive(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
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
        // this is correct because of coordinate system.
        double vx = Helpers.applySensitivityFactor(_oi.getDriveY(), SENSITIVITY_VX) * MAX_MPS;
        double vy = Helpers.applySensitivityFactor(-_oi.getDriveX(), SENSITIVITY_VY) * MAX_MPS;
        metric("vx", vx);
        metric("vy", vy);
        double rot =
                Helpers.applySensitivityFactor(-_oi.getRotationX(), SENSITIVITY_OMEGA)
                        * MAX_ANG_VEL;
        _driveTrain.drive(vx, vy, rot, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
