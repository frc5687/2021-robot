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

public class StealTenBallAuto extends SequentialCommandGroup {
    public StealTenBallAuto(
            DriveTrain driveTrain,
            Shooter shooter,
            Hood hood,
            Intake intake,
            Spindexer spindexer,
            Trajectory prt1,
            Trajectory prt2,
            Trajectory prt3,
            Trajectory exit,
            OI oi) {
        addRequirements(driveTrain, shooter);
        addCommands(
                new SetShooterSetpoint(shooter, hood, Constants.Hood.MIN_ANGLE, 4000),
                new InstantCommand(shooter::setShooterFromReference, shooter),
                new ParallelDeadlineGroup(
                        new AutoIntake(intake),
                        new DriveTrajectory(driveTrain, prt1, Rotation2d.fromDegrees(0))),
                new DriveTrajectory(driveTrain, prt2, Rotation2d.fromDegrees(0)),
                new ParallelDeadlineGroup(
                        new AutoShoot(shooter, spindexer, hood),
                        new AutoTarget(driveTrain, shooter, hood, oi, 60, 5000, true)),
                new ParallelDeadlineGroup(
                        new DriveTrajectory(driveTrain, prt3, Rotation2d.fromDegrees(0)),
                        new AutoIntake(intake)),
                new AutoAlign(driveTrain, 0),
                new DriveTrajectory(driveTrain, exit, Rotation2d.fromDegrees(0)),
                new ParallelDeadlineGroup(
                        new AutoShoot(shooter, spindexer, hood),
                        new AutoTarget(driveTrain, shooter, hood, oi, 65, 5000, true)));
    }
}
