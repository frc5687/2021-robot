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
        //        startPeriodic(0.005);
    }

    @Override
    public void execute() {
        super.execute();
        double stickX = _oi.getDriveY() * MAX_MPS;
        double stickY = _oi.getDriveX() * MAX_MPS;
        double rot = _oi.getRotationX() * MAX_ANG_VEL;
        _driveTrain.drive(stickX, stickY, rot, true);
        //        Vector2d drive = new Vector2d(stickX, stickY);
        //        metric("Drive Magnitude", drive.getMagnitude());
        //        metric("Drive Angle", drive.getAngle());
        //        _driveTrain.setBackLeftModuleVector(drive);
        //        _driveTrain.setFrontRightModuleVector(drive);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
