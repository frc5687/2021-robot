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
        double speed = 0;
        _hood.setSpeed(speed);
        if (_hood.isBottomHallTriggered()) {
            if (speed < 0) {
                _hood.setSpeed(0);
            } else if (speed > 0) {
                _hood.setSpeed(speed);
            }
        } else if (_hood.isTopHallTriggered()) {
            if (speed > 0) {
                _hood.setSpeed(0);
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
