/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.subsystems.Intake;

public class IdleIntake extends OutliersCommand {
    private Intake _intake;

    public IdleIntake(Intake intake) {
        _intake = intake;
        addRequirements(_intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        //        _intake.lowerIntake();
        //        _intake.raiseIntake();
    }

    @Override
    public void execute() {
        _intake.raiseIntake(); //Make sure the intake is up
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
