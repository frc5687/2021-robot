/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.subsystems.Climber;

public class Climb extends OutliersCommand {
    private Climber _climber;

    public Climb(Climber climber) {
        _climber = climber;
        addRequirements(_climber);
    }

    @Override
    public void initialize() {
        _climber.raiseArm();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return _climber.getArmPosition() == Climber.Position.RAISED;
    }
}
