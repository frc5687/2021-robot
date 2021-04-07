/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import org.frc5687.infiniterecharge.robot.util.SwerveTrajectory;

public class DriveMultipleTrajectory extends SequentialCommandGroup {
    public DriveMultipleTrajectory(
            DriveTrain driveTrain,
            SwerveTrajectory trajectory1,
            SwerveTrajectory trajectory2,
            SwerveTrajectory trajectory3,
            SwerveTrajectory trajectory4) {
        addRequirements(driveTrain);
        addCommands(
                new DriveTrajectory(driveTrain, trajectory1),
                new DriveTrajectory(driveTrain, trajectory2),
                new DriveTrajectory(driveTrain, trajectory3),
                new DriveTrajectory(driveTrain, trajectory4));
    }
}
