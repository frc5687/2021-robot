package org.frc5687.infiniterecharge.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.infiniterecharge.robot.commands.AutoIntake;
import org.frc5687.infiniterecharge.robot.commands.AutoShoot;
import org.frc5687.infiniterecharge.robot.commands.AutoTarget;
import org.frc5687.infiniterecharge.robot.commands.DriveTrajectory;
import org.frc5687.infiniterecharge.robot.subsystems.*;

public class StealBallAuto extends SequentialCommandGroup {
    public StealBallAuto(
            DriveTrain driveTrain,
            Shooter shooter,
            Hood hood,
            Intake intake,
            Spindexer spindexer,
            Trajectory trajectory,
            Trajectory exit) {
        addRequirements(driveTrain, shooter);
        addCommands(
                new ParallelDeadlineGroup(
                        new AutoShoot(shooter, spindexer),
                        new AutoTarget(driveTrain, shooter, hood)),
                new ParallelDeadlineGroup(
                        new DriveTrajectory(driveTrain, trajectory),
                        new AutoIntake(intake)),
                new DriveTrajectory(driveTrain, exit, Rotation2d.fromDegrees(0)),
                new ParallelDeadlineGroup(
                        new AutoShoot(shooter, spindexer),
                        new AutoTarget(driveTrain, shooter, hood)));
    }
}
