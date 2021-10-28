/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.Intake;

public class AutoIntake extends OutliersCommand {
    private Intake _intake;

    public AutoIntake(Intake intake) {
        _intake = intake;
        addRequirements(_intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        _intake.lowerIntake();
    }

    @Override
    public void execute() {
        _intake.setRollerSpeed(Constants.Intake.INTAKE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _intake.setRollerSpeed(0.0);
        _intake.raiseIntake();
    }
}
