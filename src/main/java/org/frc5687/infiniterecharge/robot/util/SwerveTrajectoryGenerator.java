/* (C)2021 */
package org.frc5687.infiniterecharge.robot.util;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import java.util.ArrayList;
import java.util.List;

public class SwerveTrajectoryGenerator {

    private SwerveTrajectoryGenerator() {}

    public static SwerveTrajectory generateTrajectory(
            List<Pose2d> waypoints, List<Rotation2d> headings, TrajectoryConfig config) {
        return new SwerveTrajectory(
                TrajectoryGenerator.generateTrajectory(waypoints, config), waypoints, headings);
    }

    public static SwerveTrajectory generateTrajectory(
            List<Pose2d> waypoints, TrajectoryConfig config, Transform2d transform) {
        List<Pose2d> newWaypoints = new ArrayList<>();
        waypoints.forEach(
                (pose2d -> {
                    newWaypoints.add(pose2d.transformBy(transform));
                }));
        return new SwerveTrajectory(
                TrajectoryGenerator.generateTrajectory(newWaypoints, config), waypoints);
    }

    public static SwerveTrajectory generateTrajectory(
            List<Pose2d> waypoints,
            List<Rotation2d> headings,
            TrajectoryConfig config,
            Transform2d transform) {
        List<Pose2d> newWaypoints = new ArrayList<>();
        waypoints.forEach(
                (pose2d -> {
                    newWaypoints.add(pose2d.transformBy(transform));
                }));
        return new SwerveTrajectory(
                TrajectoryGenerator.generateTrajectory(newWaypoints, config), waypoints, headings);
    }
}
