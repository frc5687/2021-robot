/* (C)2020-2021 */
package org.frc5687.infiniterecharge.robot.commands;

import static org.frc5687.infiniterecharge.robot.Constants.DriveTrain.*;

import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;

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
        double vx = _oi.getDriveY() * MAX_MPS;
        double vy = -_oi.getDriveX() * MAX_MPS;
        metric("vx", vx);
        metric("vy", vy);
        double rot = -_oi.getRotationX() * MAX_ANG_VEL;
        _driveTrain.drive(vx, vy, rot, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
