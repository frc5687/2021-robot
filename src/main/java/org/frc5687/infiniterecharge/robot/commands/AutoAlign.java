/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;

public class AutoAlign extends PIDCommand {
    public AutoAlign(DriveTrain drivetrain, double angle) {
        super(
                new PIDController(
                        Constants.DriveTrain.ANGLE_kP,
                        Constants.DriveTrain.ANGLE_kI,
                        Constants.DriveTrain.ANGLE_kD),
                drivetrain::getYaw,
                angle,
                output -> drivetrain.drive(0, 0, output, false),
                drivetrain);

        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(1.0);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
