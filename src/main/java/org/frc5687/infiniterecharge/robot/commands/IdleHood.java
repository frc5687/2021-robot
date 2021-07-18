/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands;

import static org.frc5687.infiniterecharge.robot.Constants.Hood.MAX_ANGLE;
import static org.frc5687.infiniterecharge.robot.Constants.Hood.MIN_ANGLE;

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
        //        metric("Hood ANgle", _hood.getAngle());
        double speed = 0;
        _hood.setSpeed(speed);
        metric("Output", _hood.getOutput());
        if (_hood.isHallTriggered()) {
            if (speed < 0) {
                _hood.setSpeed(0);
                _hood.setEncoderAngle(MIN_ANGLE);
            } else if (speed > 0) {
                _hood.setSpeed(speed);
            }
        } else if (_hood.isTopHallTriggered()) {
            if (speed > 0) {
                _hood.setSpeed(0);
                _hood.setEncoderAngle(MAX_ANGLE);
            } else if (speed < 0) {
                _hood.setSpeed(speed);
            }
        } else {
            _hood.setSpeed(speed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
