/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.commands.AutoTarget;
import org.frc5687.infiniterecharge.robot.commands.DriveTrajectory;
import org.frc5687.infiniterecharge.robot.commands.shooter.AutoShoot;
import org.frc5687.infiniterecharge.robot.commands.shooter.SetShooterSetpoint;
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
                new SetShooterSetpoint(shooter, hood, 64, 4500), //new SetShooterSetpoint(shooter, hood, 54, 4500),
                new InstantCommand(shooter::setShooterFromReference, shooter),
                new ParallelDeadlineGroup(
                        new AutoShoot(shooter, spindexer, hood),
                        new AutoTarget(driveTrain, shooter, hood, oi, 50, 5000, true))); // used to be angle of 70 rpm 4500
    }
}