/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands.climber;

import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.commands.OutliersCommand;
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
        _climber.setWinchSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
