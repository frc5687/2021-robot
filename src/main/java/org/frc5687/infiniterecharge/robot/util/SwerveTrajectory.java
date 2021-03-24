/* (C)2021 */
package org.frc5687.infiniterecharge.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import java.util.Collections;
import java.util.List;

public class SwerveTrajectory extends Trajectory {

    private final List<Pose2d> waypoints;
    private List<Rotation2d> headings;
    private List<Trajectory.State> states;
    private final double totalTimeSeconds;

    private boolean prevEquals = false;
    private int n = 0;

    public SwerveTrajectory(Trajectory trajectory, List<Pose2d> waypoints) {
        this.states = trajectory.getStates();
        this.waypoints = waypoints;
        this.totalTimeSeconds = trajectory.getTotalTimeSeconds();
        this.headings = Collections.singletonList(new Rotation2d(0));
    }

    public SwerveTrajectory(
            Trajectory trajectory, List<Pose2d> waypoints, List<Rotation2d> headings) {
        this(trajectory, waypoints);
        this.headings = headings;
    }

//    /**
//     * Sample the trajectory at a point in time.
//     *
//     * @param timeSeconds The point in time since the beginning of the trajectory to sample.
//     * @return The state at that point in time.
//     */
//    public Trajectory.State sample(double timeSeconds) {
//        if (timeSeconds <= states.get(0).timeSeconds) {
//            return states.get(0);
//        }
//        if (timeSeconds >= totalTimeSeconds) {
//            return states.get(states.size() - 1);
//        }
//
//        // To get the element that we want, we will use a binary search algorithm
//        // instead of iterating over a for-loop. A binary search is O(std::log(n))
//        // whereas searching using a loop is O(n).
//
//        // This starts at 1 because we use the previous state later on for
//        // interpolation.
//        int low = 1;
//        int high = states.size() - 1;
//
//        while (low != high) {
//            int mid = (low + high) / 2;
//            if (states.get(mid).timeSeconds < timeSeconds) {
//                // This index and everything under it are less than the requested
//                // timestamp. Therefore, we can discard them.
//                low = mid + 1;
//            } else {
//                // t is at least as large as the element at this index. This means that
//                // anything after it cannot be what we are looking for.
//                high = mid;
//            }
//        }
//
//        // High and Low should be the same.
//
//        // The sample's timestamp is now greater than or equal to the requested
//        // timestamp. If it is greater, we need to interpolate between the
//        // previous state and the current state to get the exact state that we
//        // want.
//        final Trajectory.State sample = states.get(low);
//        final Trajectory.State prevSample = states.get(low - 1);
//
//        // If the difference in states is negligible, then we are spot on!
//        if (Math.abs(sample.timeSeconds - prevSample.timeSeconds) < 1E-9) {
//            return sample;
//        }
//        // Interpolate between the two states for the state that we want.
//        return prevSample.interpolate(
//                sample,
//                (timeSeconds - prevSample.timeSeconds) / (sample.timeSeconds - prevSample.timeSeconds));
//    }
//    public State sample(double timeSeconds) {
//        boolean equals =
//                equals(
//                        trajectory.sample(timeSeconds).poseMeters.getTranslation(),
//                        waypoints.get(n).getTranslation());
//
//        if (equals && !prevEquals) {
//            if (n != headings.size() - 1) {
//                DriverStation.reportError(
//                        trajectory.sample(timeSeconds).poseMeters.toString()
//                                + " is equal to "
//                                + waypoints.get(n).toString(),
//                        false);
//                n++;
//            }
//            prevEquals = equals;
//        }
//        if (waypoints.size() != headings.size()) {
//            return new State(trajectory.sample(timeSeconds), new Rotation2d(0));
//        }
//        prevEquals = equals;
//        return new State(trajectory.sample(timeSeconds), headings.get(n));
//    }

    private Rotation2d lerp(Rotation2d startVal, Rotation2d endVal, double t) {
        return startVal.plus((endVal.minus(startVal)).times(t));
    }

//    public double getTotalTimeSeconds() {
//        return trajectory.getTotalTimeSeconds();
//    }

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
