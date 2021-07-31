/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.subsystems.Climber;

public class RaiseArm extends OutliersCommand {
    private Climber _climber;

    public RaiseArm(Climber climber) {
        _climber = climber;
    }

    @Override
    public void initialize() {
        super.initialize();
        _climber.raiseArm();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
