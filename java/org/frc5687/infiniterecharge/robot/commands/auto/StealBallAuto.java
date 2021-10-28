/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.commands.*;
import org.frc5687.infiniterecharge.robot.commands.shooter.AutoShoot;
import org.frc5687.infiniterecharge.robot.subsystems.*;

public class StealBallAuto extends SequentialCommandGroup {
    public StealBallAuto(DriveTrain driveTrain,
            Shooter shooter,
            Hood hood,
            Intake intake,
            Spindexer spindexer,
            Trajectory trajectory,
            Trajectory exit,
            OI oi) {
        addCommands(
                new ParallelDeadlineGroup(new AutoIntake(intake), new DriveTrajectory(driveTrain, trajectory)),
                new DriveTrajectory(driveTrain, exit, Rotation2d.fromDegrees(0)),
                new ParallelDeadlineGroup(new AutoShoot(shooter, spindexer, hood), new AutoTarget(driveTrain, shooter, hood, oi, 0, 0, false)));
    }
}
