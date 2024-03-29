/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands.climber;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.commands.OutliersCommand;
import org.frc5687.infiniterecharge.robot.subsystems.Climber;

public class Climb extends OutliersCommand {
    private Climber _climber;

    public Climb(Climber climber) {
        _climber = climber;
        addRequirements(_climber);
    }

    public void initialize() {
//        _climber.lowerArm();
    }

    @Override
    public void execute() {
        _climber.setWinchSpeed(Constants.Climber.WINCH_SPEED);
    }

    @Override
    public boolean isFinished() {
        //The values are switched LOWERED means up
//        return _climber.getArmPosition() == Climber.Position.RAISED;
        return false;
    }
}
