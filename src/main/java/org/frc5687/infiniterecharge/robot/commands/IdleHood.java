/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.Hood;

public class IdleHood extends OutliersCommand {

    private Hood _hood;
    private OI _oi;

    public IdleHood(Hood hood, OI oi) {
        _hood = hood;
        _oi = oi;
        addRequirements(_hood);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        metric("Hood ANgle", _hood.getAngle());
        if (_hood.isHallTriggered() && _hood.getOutput() < 0) {
            _hood.setSpeed(0);
            _hood.setEncoderAngle(20);
        } else {
            _hood.setSpeed(_oi.getDriveY());
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
