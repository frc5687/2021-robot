/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.commands.*;
import org.frc5687.infiniterecharge.robot.commands.shooter.AutoShoot;
import org.frc5687.infiniterecharge.robot.commands.shooter.SetShooterSetpoint;
import org.frc5687.infiniterecharge.robot.subsystems.*;

public class TenBallAuto extends SequentialCommandGroup {
    public TenBallAuto(
            DriveTrain driveTrain,
            Shooter shooter,
            Hood hood,
            Intake intake,
            Spindexer spindexer,
            Trajectory prtOne,
            Trajectory prtTwo,
            Trajectory prtThree,
            Trajectory exit,
            OI oi) {
        addRequirements(driveTrain, shooter);
        addCommands(
                new SetShooterSetpoint(shooter, hood, Constants.Hood.MIN_ANGLE, 3000),
                new InstantCommand(shooter::setShooterFromReference, shooter),
                new ParallelDeadlineGroup(
                        new DriveTrajectory(driveTrain, prtOne, Rotation2d.fromDegrees(120)),
                        new AutoIntake(intake)),
                new DriveTrajectory(driveTrain, prtTwo, Rotation2d.fromDegrees(0)),
                new ParallelDeadlineGroup(
                        new AutoShoot(shooter, spindexer, hood),
                        new AutoTarget(driveTrain, shooter, hood, oi, 0, 0, false)),
                new ParallelDeadlineGroup(
                        new DriveTrajectory(driveTrain, prtThree, Rotation2d.fromDegrees(0)),
                        new AutoIntake(intake)),
                new DriveTrajectory(driveTrain, exit, Rotation2d.fromDegrees(0)),
                new ParallelDeadlineGroup(
                        new AutoShoot(shooter, spindexer, hood),
                        new AutoTarget(driveTrain, shooter, hood, oi, 0, 0, false)));
    }
}
