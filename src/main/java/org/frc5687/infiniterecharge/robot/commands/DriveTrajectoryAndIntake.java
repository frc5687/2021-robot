/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import org.frc5687.infiniterecharge.robot.subsystems.Intake;
import org.frc5687.infiniterecharge.robot.util.SwerveTrajectory;

public class DriveTrajectoryAndIntake extends ParallelCommandGroup {
    public DriveTrajectoryAndIntake(
            DriveTrain driveTrain, Intake intake, SwerveTrajectory trajectory) {
        addCommands(new DriveTrajectory(driveTrain, trajectory), new AutoIntake(intake));
    }
}
