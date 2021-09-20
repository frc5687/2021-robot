/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands.shooter;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.commands.OutliersCommand;
import org.frc5687.infiniterecharge.robot.subsystems.Hood;

public class IdleHood extends OutliersCommand {

    private Hood _hood;
    private OI _oi;

    private boolean _zeroing = false;

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
        if (_zeroing) {
            _hood.setSpeed(Constants.Hood.ZEROING_SPEED);
            if (_hood.isBottomHallTriggered()) {
                _hood.setPosition(Constants.Hood.MIN_ANGLE);
                _zeroing = false;
            }
        } else {
            _hood.setSpeed(0);
        }
    }

    public void setZeroing(boolean value) {
        _zeroing = value;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
