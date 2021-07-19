/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.Climber;

public class IdleClimber extends OutliersCommand {

    private Climber _climber;
    private OI _oi;

    public IdleClimber(Climber climber, OI oi) {
        _climber = climber;
        _oi = oi;
        addRequirements(_climber);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double speed = _oi.getWinchSpeed();
        _climber.setWinchSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
