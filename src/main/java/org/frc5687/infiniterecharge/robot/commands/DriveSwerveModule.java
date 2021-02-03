/* (C)2020-2021 */
package org.frc5687.infiniterecharge.robot.commands;

import static org.frc5687.infiniterecharge.robot.Constants.DriveTrain.*;

import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;

public class DriveSwerveModule extends OutliersCommand {

    private DriveTrain _driveTrain;
    private OI _oi;

    public DriveSwerveModule(DriveTrain driveTrain, OI oi) {
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
        double stickX = _oi.getDriveY() * MAX_MPS;
        double stickY = _oi.getDriveX() * MAX_MPS;
        double rot = _oi.getRotationX() * MAX_ANG_VEL;
        _driveTrain.drive(stickX, stickY, rot, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
