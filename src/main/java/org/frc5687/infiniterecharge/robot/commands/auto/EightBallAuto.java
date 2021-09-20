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

public class EightBallAuto extends SequentialCommandGroup {
    public EightBallAuto(
            DriveTrain driveTrain,
            Shooter shooter,
            Hood hood,
            Intake intake,
            Spindexer spindexer,
            Trajectory trajectory,
            Trajectory exit,
            OI oi) {
        addRequirements(driveTrain, shooter);
        addCommands(
                new SetShooterSetpoint(shooter, hood, Constants.Hood.MIN_ANGLE, 3000),
                new InstantCommand(shooter::setShooterFromReference, shooter),
                new ParallelDeadlineGroup(
                        new AutoShoot(shooter, spindexer, hood),
                        new AutoTarget(driveTrain, shooter, hood, oi, 52, 4500, true)),
                new ParallelDeadlineGroup(
                        new DriveTrajectory(driveTrain, trajectory), new AutoIntake(intake)),
                new DriveTrajectory(driveTrain, exit, Rotation2d.fromDegrees(0)),
                new ParallelDeadlineGroup(
                        new AutoShoot(shooter, spindexer, hood),
                        new AutoTarget(driveTrain, shooter, hood, oi, 63, 5000, false)));
    }
}
