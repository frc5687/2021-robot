/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands.shooter;

import org.frc5687.infiniterecharge.robot.commands.OutliersCommand;
import org.frc5687.infiniterecharge.robot.subsystems.Hood;

public class ZeroHood extends OutliersCommand {

    private Hood _hood;

    public ZeroHood(Hood hood) {
        _hood = hood;
        addRequirements(_hood);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        if (!_hood.isBottomHallTriggered()) {
            _hood.setSpeed(-0.5);
        } else {
            _hood.zeroSensors();
        }
    }

    @Override
    public boolean isFinished() {
        return _hood.isBottomHallTriggered();
    }
}
