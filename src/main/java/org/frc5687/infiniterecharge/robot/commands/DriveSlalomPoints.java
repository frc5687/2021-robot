/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;

public class DriveSlalomPoints extends SequentialCommandGroup {

    public DriveSlalomPoints(DriveTrain driveTrain) {
        addCommands(
                new DriveToPoint(driveTrain, Constants.AutoPaths.Slalom.waypoints.get(0)),
                new DriveToPoint(driveTrain, Constants.AutoPaths.Slalom.waypoints.get(1)),
                new DriveToPoint(driveTrain, Constants.AutoPaths.Slalom.waypoints.get(2)),
                new DriveToPoint(driveTrain, Constants.AutoPaths.Slalom.waypoints.get(3)),
                new DriveToPoint(driveTrain, Constants.AutoPaths.Slalom.waypoints.get(4)),
                new DriveToPoint(driveTrain, Constants.AutoPaths.Slalom.waypoints.get(5)),
                new DriveToPoint(driveTrain, Constants.AutoPaths.Slalom.waypoints.get(6)),
                new DriveToPoint(driveTrain, Constants.AutoPaths.Slalom.waypoints.get(7)),
                new DriveToPoint(driveTrain, Constants.AutoPaths.Slalom.waypoints.get(8)),
                new DriveToPoint(driveTrain, Constants.AutoPaths.Slalom.waypoints.get(9)),
                new DriveToPoint(driveTrain, Constants.AutoPaths.Slalom.waypoints.get(10)),
                new DriveToPoint(driveTrain, Constants.AutoPaths.Slalom.waypoints.get(11)));
    }
}
