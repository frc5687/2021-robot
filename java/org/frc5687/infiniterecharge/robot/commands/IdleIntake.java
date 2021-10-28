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
        _intake.raiseIntake();
        _intake.setRollerSpeed(0.0);
    }

    @Override
    public void execute() {
        _intake.raiseIntake(); //Make sure the intake is up
        _intake.setRollerSpeed(0.0);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
