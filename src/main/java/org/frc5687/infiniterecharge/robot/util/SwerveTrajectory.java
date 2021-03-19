/* (C)2021 */
package org.frc5687.infiniterecharge.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import java.util.Collections;
import java.util.List;

public class SwerveTrajectory {

    private final Trajectory trajectory;
    private final List<Pose2d> waypoints;
    private final List<Rotation2d> headings;

    private boolean prevEquals = false;
    private int n = 0;

    public SwerveTrajectory(Trajectory trajectory, List<Pose2d> waypoints) {
        this.trajectory = trajectory;
        this.waypoints = waypoints;
        this.headings = Collections.singletonList(new Rotation2d(0));
    }

    public SwerveTrajectory(
            Trajectory trajectory, List<Pose2d> waypoints, List<Rotation2d> headings) {
        this.trajectory = trajectory;
        this.waypoints = waypoints;
        this.headings = headings;
    }

    public State sample(double timeSeconds) {
        boolean equals =
                equals(
                        trajectory.sample(timeSeconds).poseMeters.getTranslation(),
                        waypoints.get(n).getTranslation());

        if (equals && !prevEquals) {
            if (n != headings.size() - 1) {
                DriverStation.reportError(
                        trajectory.sample(timeSeconds).poseMeters.toString()
                                + " is equal to "
                                + waypoints.get(n).toString(),
                        false);
                n++;
            }
            prevEquals = equals;
        }
        if (waypoints.size() != headings.size()) {
            return new State(trajectory.sample(timeSeconds), new Rotation2d(0));
        }
        prevEquals = equals;
        return new State(trajectory.sample(timeSeconds), headings.get(n));
    }

    public double getTotalTimeSeconds() {
        return trajectory.getTotalTimeSeconds();
    }

    public boolean equals(Translation2d translation1, Translation2d translation2) {
        return (Math.abs(translation1.getX() - translation2.getX()) < 0.01
                && Math.abs(translation1.getY() - translation2.getY()) < 0.01);
    }

    public Pose2d getInitialPose() {
        return sample(0).poseMeters;
    }

    public static class State extends Trajectory.State {
        public Trajectory.State state;
        public Rotation2d heading;

        public State(Trajectory.State state, Rotation2d heading) {
            this.state = state;
            this.heading = heading;
        }
    }
}
