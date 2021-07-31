/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.commands.AutoShoot;
import org.frc5687.infiniterecharge.robot.commands.AutoTarget;
import org.frc5687.infiniterecharge.robot.commands.DriveTrajectory;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import org.frc5687.infiniterecharge.robot.subsystems.Hood;
import org.frc5687.infiniterecharge.robot.subsystems.Shooter;
import org.frc5687.infiniterecharge.robot.subsystems.Spindexer;

public class ShootAndGo extends SequentialCommandGroup {
    public ShootAndGo(
            DriveTrain driveTrain,
            Shooter shooter,
            Hood hood,
            Spindexer spindexer,
            Trajectory traj,
            OI oi) {
        addCommands(
                new ParallelDeadlineGroup(
                        new AutoShoot(shooter, spindexer),
                        new AutoTarget(driveTrain, shooter, hood, oi, 52, 4500, true)),
                new DriveTrajectory(driveTrain, traj, driveTrain.getHeading()));
    }
}
