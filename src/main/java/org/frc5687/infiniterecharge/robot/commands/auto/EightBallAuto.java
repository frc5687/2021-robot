/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.infiniterecharge.robot.Constants;
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
                        new AutoTarget(driveTrain, shooter, hood)),
                new ParallelDeadlineGroup(
                        new AutoIntake(intake),
                        new DriveTrajectory(driveTrain, trajectory),
                        new AutoIntake(intake)),
                new DriveToPose(
                        driveTrain,
                        new Pose2d(
                                Constants.Field.START_LINE_X - Units.inchesToMeters(122.62),
                                Constants.Field.MID_TRENCH_Y,
                                new Rotation2d(0)),
                        new Rotation2d(0)),
                new ParallelDeadlineGroup(
                        new AutoShoot(shooter, spindexer),
                        new AutoTarget(driveTrain, shooter, hood)));
    }
}
