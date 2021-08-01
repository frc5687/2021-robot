/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands.shooter;

import org.frc5687.infiniterecharge.robot.commands.OutliersCommand;
import org.frc5687.infiniterecharge.robot.subsystems.Hood;

public class HoodSetpoint extends OutliersCommand {

    private Hood _hood;
    private double _setpoint;

    public HoodSetpoint(Hood hood, double setpoint) {
        _hood = hood;
        _setpoint = setpoint;
        addRequirements(_hood);
    }

    @Override
    public void initialize() {
        _hood.setPosition(_setpoint);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
