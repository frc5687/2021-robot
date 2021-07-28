package org.frc5687.infiniterecharge.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.infiniterecharge.robot.commands.*;
import org.frc5687.infiniterecharge.robot.subsystems.*;

public class EightBallAuto extends SequentialCommandGroup {
    public EightBallAuto(
            DriveTrain driveTrain,
            Shooter shooter,
            Hood hood,
            Intake intake,
            Spindexer spindexer,
            Trajectory trajectory) {
        addRequirements(driveTrain, shooter);
        addCommands(
                new ParallelDeadlineGroup(
                        new AutoShoot(shooter, spindexer),
                        new AutoTarget(driveTrain, shooter, hood)
                ),
                new ParallelDeadlineGroup(
                        new DriveTrajectory(driveTrain,trajectory),
                        new AutoIntake(intake)
                )
        );
    }
}
