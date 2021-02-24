/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.subsystems.Hood;

public class AutoHoodSetpoint extends OutliersCommand {

    private Hood _hood;

    private double _angleSetpoint;

    public AutoHoodSetpoint(Hood hood, double setpoint) {
        _hood = hood;
        _angleSetpoint = setpoint;
        addRequirements(_hood);
    }

    @Override
    public void initialize() {
        _hood.setHoodAngle(_angleSetpoint);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
