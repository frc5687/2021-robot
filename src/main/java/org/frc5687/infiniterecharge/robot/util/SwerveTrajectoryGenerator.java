/* (C)2021 */
package org.frc5687.infiniterecharge.robot.util;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.spline.*;
import edu.wpi.first.wpilibj.spline.SplineParameterizer.MalformedSplineException;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.function.BiConsumer;

public final class SwerveTrajectoryGenerator {
    private static final SwerveTrajectory kDoNothingSwerveTrajectory =
            new SwerveTrajectory(Arrays.asList(new SwerveTrajectory.State()));
    private static BiConsumer<String, StackTraceElement[]> errorFunc;

    private static final List<Rotation2d> interpolatedHeading = new ArrayList<Rotation2d>();

    /** Private constructor because this is a utility class. */
    private SwerveTrajectoryGenerator() {}

    private static void reportError(String error, StackTraceElement[] stackTrace) {
        if (errorFunc != null) {
            errorFunc.accept(error, stackTrace);
        } else {
            MathSharedStore.reportError(error, stackTrace);
        }
    }

    /**
     * Set error reporting function. By default, DriverStation.reportError() is used.
     *
     * @param func Error reporting function, arguments are error and stackTrace.
     */
    public static void setErrorHandler(BiConsumer<String, StackTraceElement[]> func) {
        errorFunc = func;
    }

    /**
     * Generates a trajectory from the given waypoints and config. This method uses quintic hermite
     * splines -- therefore, all points must be represented by Pose2d objects. Continuous curvature
     * is guaranteed in this method.
     *
     * @param waypoints List of waypoints..
     * @param config The configuration for the trajectory.
     * @return The generated trajectory.
     */
    @SuppressWarnings("LocalVariableName")
    public static SwerveTrajectory generateTrajectory(
            List<Pose2d> waypoints, List<Rotation2d> headings, TrajectoryConfig config) {
        final var flip = new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180.0));

        List<Pose2d> newWaypoints = new ArrayList<>();
        if (config.isReversed()) {
            for (Pose2d originalWaypoint : waypoints) {
                newWaypoints.add(originalWaypoint.plus(flip));
            }
        } else {
            newWaypoints.addAll(waypoints);
        }

        // Get the spline points
        List<PoseWithCurvature> points;
        try {
            points =
                    splinePointsFromSplines(
                            SplineHelper.getQuinticSplinesFromWaypoints(newWaypoints), headings);
            if (interpolatedHeading.size() != points.size()) {
                DriverStation.reportError(
                        "size of headings does not match points size. Headings size is: "
                                + headings.size()
                                + " points size is: "
                                + points.size(),
                        false);
            }
        } catch (MalformedSplineException ex) {
            reportError(ex.getMessage(), ex.getStackTrace());
            return kDoNothingSwerveTrajectory;
        }

        // Change the points back to their original orientation.
        if (config.isReversed()) {
            for (var point : points) {
                point.poseMeters = point.poseMeters.plus(flip);
                point.curvatureRadPerMeter *= -1;
            }
        }
        // Generate and return trajectory.
        return SwerveTrajectoryParameterizer.timeParameterizeTrajectory(
                points,
                interpolatedHeading,
                config.getConstraints(),
                config.getStartVelocity(),
                config.getEndVelocity(),
                config.getMaxVelocity(),
                config.getMaxAcceleration(),
                config.isReversed());
    }

    /**
     * Generate spline points from a vector of splines by parameterizing the splines.
     *
     * @param splines The splines to parameterize.
     * @return The spline points for use in time parameterization of a trajectory.
     * @throws MalformedSplineException When the spline is malformed (e.g. has close adjacent points
     *     with approximately opposing headings)
     */
    public static List<PoseWithCurvature> splinePointsFromSplines(
            Spline[] splines, List<Rotation2d> headings) {
        // Create the vector of spline points.
        var splinePoints = new ArrayList<PoseWithCurvature>();

        // Add the first point to the vector.
        splinePoints.add(splines[0].getPoint(0.0));
        interpolatedHeading.add(headings.get(0));

        // Iterate through the vector and parameterize each spline, adding the
        // parameterized points to the final vector.
        int i = 0;
        for (final var spline : splines) {
            var points = SplineParameterizer.parameterize(spline);
            var headingPoints = new ArrayList<Rotation2d>();
            for (int j = 0; j < points.size(); j++) {
                double frac = (double) j / ((double) points.size() - 1.0);
                headingPoints.add(
                        headings.get(i)
                                .plus((headings.get(i + 1).minus(headings.get(i))).times(frac)));
            }
            // Append the array of poses to the vector. We are removing the first
            // point because it's a duplicate of the last point from the previous
            // spline.
            interpolatedHeading.addAll(headingPoints.subList(1, headingPoints.size()));
            splinePoints.addAll(points.subList(1, points.size()));
            i++;
        }
        return splinePoints;
    }

    // Work around type erasure signatures
    @SuppressWarnings("serial")
    public static class ControlVectorList extends ArrayList<Spline.ControlVector> {
        public ControlVectorList(int initialCapacity) {
            super(initialCapacity);
        }

        public ControlVectorList() {
            super();
        }

        public ControlVectorList(Collection<? extends Spline.ControlVector> collection) {
            super(collection);
        }
    }
}
