/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands.climber;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.commands.OutliersCommand;
import org.frc5687.infiniterecharge.robot.subsystems.Climber;

public class ResetWinch extends OutliersCommand {

    private Climber _climber;

    public ResetWinch(Climber climber) {
        _climber = climber;
        addRequirements(_climber);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        _climber.setWinchSpeed(Constants.Climber.WINCH_RESET_SPEED);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
