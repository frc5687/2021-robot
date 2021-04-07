/* (C)2020-2021 */
package org.frc5687.infiniterecharge.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.*;
import java.util.Arrays;
import java.util.List;

public class Constants {
    public static final int TICKS_PER_UPDATE = 1;
    public static final double METRIC_FLUSH_PERIOD = 1.0;
    public static final double UPDATE_PERIOD = 0.02;
    public static final double EPSILON = 0.001;

    public static class DriveTrain {

        public static final Transform2d SLAM_TO_ROBOT =
                new Transform2d(
                        new Translation2d(0, 0),
                        new Rotation2d(0)); // TODO: Figure out real values.
        public static final Transform2d CAMERA_TO_ROBOT =
                new Transform2d(
                        new Translation2d(Units.inchesToMeters(13.75), Units.inchesToMeters(5.75)),
                        new Rotation2d(0));
        public static final Translation2d TARGET_POS =
                new Translation2d(Units.inchesToMeters(93), Units.inchesToMeters(111));

        public static final double T265_MEASUREMENT_COVARIANCE = 0.5;

        public static final double WIDTH = 0.619125;
        public static final double LENGTH = 0.619125;
        public static final Translation2d FRONT_LEFT_POSITION =
                new Translation2d(WIDTH / 2.0, LENGTH / 2.0);
        public static final Translation2d FRONT_RIGHT_POSITION =
                new Translation2d(WIDTH / 2.0, -LENGTH / 2.0);
        public static final Translation2d BACK_LEFT_POSITION =
                new Translation2d(-WIDTH / 2.0, LENGTH / 2.0);
        public static final Translation2d BACK_RIGHT_POSITION =
                new Translation2d(-WIDTH / 2.0, -LENGTH / 2.0);

        // units are meters and radians.
        // System pose [x, y, theta] certainty TODO: Tune for real bot.
        public static final Matrix<N3, N1> STATE_STD_DEVS =
                VecBuilder.fill(0.3, 0.3, Units.degreesToRadians(5));
        // Heading certainty TODO: Tune for real bot.
        public static final Matrix<N1, N1> LOCAL_MEASUREMENT_STD_DEVS =
                VecBuilder.fill(Units.degreesToRadians(0.01));
        // Vision pose [x, y, theta] certainty TODO: Tune for real bot.
        public static final Matrix<N3, N1> VISION_MEASUREMENT_STD_DEVS =
                VecBuilder.fill(0.4, 0.4, Units.degreesToRadians(20));

        public static final double DEADBAND = 0.1;
        public static final double SENSITIVITY_VX = 0.9;
        public static final double SENSITIVITY_VY = 0.9;
        public static final double SENSITIVITY_OMEGA = 0.3;
        public static final double MAX_MPS = 4.1816;
        public static final double MAX_MPSS = 2.0; // accel

        public static final double MAX_ANG_VEL = Math.PI * 2.0;

        public static final double ANGLE_kP = 5.5;
        public static final double ANGLE_kI = 0.0;
        public static final double ANGLE_kD = 0.1;

        public static final double kP = 4.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double PROFILE_CONSTRAINT_VEL = 4.0 * Math.PI;
        public static final double PROFILE_CONSTRAINT_ACCEL = 12.0 * Math.PI;
    }

    public static class DifferentialSwerveModule {

        public static final double kDt = 0.005;

        public static final double FALCON_FREE_SPEED =
                Units.rotationsPerMinuteToRadiansPerSecond(6380);
        public static final int TIMEOUT = 200;
        public static final double GEAR_RATIO_WHEEL = 6.46875;
        public static final double GEAR_RATIO_STEER = 11.5;
        public static final double FALCON_RATE = 600.0;
        public static final double WHEEL_RADIUS = 0.0469; // Meters with compression.
        public static final double TICKS_TO_ROTATIONS = 2048.0;
        public static final double VOLTS_TO_ROTATIONS = 3.3;
        public static final double FEED_FORWARD = 12.0 / (FALCON_FREE_SPEED / GEAR_RATIO_WHEEL);
        public static final double VOLTAGE = 10.0;

        // Create Parameters for DiffSwerve State Space
        public static final double INERTIA_WHEEL = 0.005;
        public static final double INERTIA_STEER = 0.004;
        public static final double Q_AZIMUTH_ANG_VELOCITY = 0.1; // radians per sec
        public static final double Q_AZIMUTH = .01; // radians
        public static final double Q_WHEEL_ANG_VELOCITY = 0.5; // radians per sec
        public static final double MODEL_AZIMUTH_ANGLE_NOISE = .1; // radians
        public static final double MODEL_AZIMUTH_ANG_VELOCITY_NOISE = 3.0; // radians per sec
        public static final double MODEL_WHEEL_ANG_VELOCITY_NOISE = 3.0; // radians per sec
        public static final double SENSOR_AZIMUTH_ANGLE_NOISE = 0.01; // radians
        public static final double SENSOR_AZIMUTH_ANG_VELOCITY_NOISE = 0.8; // radians per sec
        public static final double SENSOR_WHEEL_ANG_VELOCITY_NOISE = 0.8; // radians per sec
        public static final double CONTROL_EFFORT = 10;
    }

    public static class Spindexer {
        public static boolean SPINDEXER_INVERTED = false;

        public static boolean FEEDER_INVERTED = false;
        public static final double SPINDEXER_IDLE_SPEED = -0.0;
        public static final double FEEDER_IDLE_SPEED = -0.25;
        public static final double SPINDEXER_SPEED = 0.75;
        public static final double FEEDER_SPEED = 1.0;
    }

    public static class Intake {
        public static final double INTAKE_SPEED = 1.0;
        public static boolean INVERTED = true;
    }

    public static class Hood {
        public static final boolean INVERTED = true;

        public static final double DISTANCE_PER_ROTATION = 2; // mm

        public static final double kP = 0.0008;
        public static final double kI = 0.00000;
        public static final double kD = 0.00;
        public static final double kFF = 0.0001;
        public static final double kIz = 1.0;

        public static final double MIN_OUTPUT = -1.0;
        public static final double MAX_OUTPUT = 1.0;

        public static final double MIN_VEL = 0;
        public static final double MAX_VEL = Units.radiansPerSecondToRotationsPerMinute(800);
        public static final double MAX_ACCEL = Units.radiansPerSecondToRotationsPerMinute(650);

        public static final double TOLERANCE = 0.02; // rads

        public static final double MIN_ANGLE = 20;
        public static final double MAX_ANGLE = 85;

        public static final double POSITION_TO_ANGLE = 1.354166666666667; // TODO
    }

    public static class Shooter {
        public static final boolean LEFT_INVERTED = true;
        public static final boolean RIGHT_INVERTED = false;

        public static final double kP = 0.4;
        public static final double kI = 0.0025;
        public static final double kD = 0.6;
        public static final double kFF = 0.05;
        public static final int kIz = 150;

        public static final double GEAR_RATIO = 1.25;
        public static final double MAX_RPM = 6380 * GEAR_RATIO;
        public static final double TICKS_TO_ROTATIONS = 2048.0;

        public static final double TOLERANCE = 100.0;
        public static final long TIMEOUT = 10500; // millis
    }

    public static class Field {
        public static final double FULL_FIELD_X = 16.0;
        public static final double HALF_FIELD_X = FULL_FIELD_X / 2.0;
        public static final double FULL_FIELD_Y = 8.21055;
        public static final double TARGET_LINE_Y = 2.404364;

        public static final Pose2d TARGET_POSITION =
                new Pose2d(FULL_FIELD_X, TARGET_LINE_Y, new Rotation2d(0));
    }

    public static class AutoPaths {
        public static class Slalom {
            public static final List<Pose2d> waypoints =
                    Arrays.asList(
                            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)), // good
                            new Pose2d(1.1, 0.75, Rotation2d.fromDegrees(60)), // good
                            new Pose2d(1.7, 1.5, Rotation2d.fromDegrees(5)), // good
                            new Pose2d(5.1, 1.5, Rotation2d.fromDegrees(-20)), // not good
                            new Pose2d(5.64, 0.75, Rotation2d.fromDegrees(-90)), // not good
                            new Pose2d(6.4, 0.0, Rotation2d.fromDegrees(0)), // not good
                            new Pose2d(7.1, 0.75, Rotation2d.fromDegrees(90)), // not good
                            new Pose2d(6.4, 1.5, Rotation2d.fromDegrees(180)),
                            new Pose2d(5.64, 0.75, Rotation2d.fromDegrees(-130)),
                            new Pose2d(5.1, 0.0, Rotation2d.fromDegrees(-170)),
                            new Pose2d(1.5, 0.0, Rotation2d.fromDegrees(170)),
                            new Pose2d(0.2, 1.8, Rotation2d.fromDegrees(180)));
            public static final List<Rotation2d> headings =
                    Arrays.asList(
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0));
            //                    Arrays.asList(
            //                            Rotation2d.fromDegrees(0),
            //                            Rotation2d.fromDegrees(60),
            //                            Rotation2d.fromDegrees(0),
            //                            Rotation2d.fromDegrees(0),
            //                            Rotation2d.fromDegrees(0),
            //                            Rotation2d.fromDegrees(90),
            //                            Rotation2d.fromDegrees(180),
            //                            Rotation2d.fromDegrees(-90),
            //                            Rotation2d.fromDegrees(-90),
            //                            Rotation2d.fromDegrees(-90),
            //                            Rotation2d.fromDegrees(-90),
            //                            Rotation2d.fromDegrees(-90),
            //                            Rotation2d.fromDegrees(-120));
        }

        public static class BouncePath {
            public static final List<Pose2d> waypoint1 =
                    Arrays.asList(
                            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)), // good
                            new Pose2d(1.10, 1.2, Rotation2d.fromDegrees(90)), // good
                            new Pose2d(1.12, 1.32, Rotation2d.fromDegrees(90)) // good
                            );

            public static final List<Pose2d> waypoint2 =
                    Arrays.asList(
                            new Pose2d(1.12, 1.32, Rotation2d.fromDegrees(-90)),
                            new Pose2d(1.5, -0.0, Rotation2d.fromDegrees(-60)),
                            new Pose2d(1.9, -0.762, Rotation2d.fromDegrees(-90)),
                            new Pose2d(2.82, -1.22, Rotation2d.fromDegrees(0)),
                            new Pose2d(3.35, -0.762, Rotation2d.fromDegrees(90)),
                            new Pose2d(3.35, 1.32, Rotation2d.fromDegrees(90)));
            public static final List<Pose2d> waypoint3 =
                    Arrays.asList(
                            new Pose2d(3.35, 1.32, Rotation2d.fromDegrees(-90)),
                            new Pose2d(3.35, -0.762, Rotation2d.fromDegrees(-90)),
                            new Pose2d(5.33, -1.22, Rotation2d.fromDegrees(0)),
                            new Pose2d(5.71, -0.76, Rotation2d.fromDegrees(90)),
                            new Pose2d(5.71, 1.72, Rotation2d.fromDegrees(90)));
            public static final List<Pose2d> waypoint4 =
                    Arrays.asList(
                            new Pose2d(5.71, 1.72, Rotation2d.fromDegrees(-90)),
                            new Pose2d(7.24, 0.5, Rotation2d.fromDegrees(0)));
            public static final List<Rotation2d> headings1 =
                    Arrays.asList(
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0));
            public static final List<Rotation2d> headings2 =
                    Arrays.asList(
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0));
            public static final List<Rotation2d> headings3 =
                    Arrays.asList(
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0));
            public static final List<Rotation2d> headings4 =
                    Arrays.asList(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0));
        }

        public static class GalacticSearch1 {
            public static final List<Pose2d> waypoints =
                    Arrays.asList(
                            new Pose2d(0, 0, new Rotation2d(0)),
                            new Pose2d(1.35, 0, new Rotation2d(0)),
                            new Pose2d(3.1, 0, new Rotation2d(0)),
                            new Pose2d(3.7, 1.35, new Rotation2d(45)),
                            new Pose2d(8.1, 1, new Rotation2d(0)));
            public static final List<Rotation2d> headings =
                    Arrays.asList(
                            Rotation2d.fromDegrees(-180),
                            Rotation2d.fromDegrees(-135),
                            Rotation2d.fromDegrees(135),
                            Rotation2d.fromDegrees(-130),
                            Rotation2d.fromDegrees(180));
        }

        public static class BarrelRun {
            public static final List<Pose2d> waypoints =
                    Arrays.asList(
                            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)),
                            new Pose2d(2.6, 0.0, Rotation2d.fromDegrees(0)),
                            new Pose2d(3.5, -0.762, Rotation2d.fromDegrees(-90)),
                            new Pose2d(2.6, -1.5, Rotation2d.fromDegrees(180)),
                            new Pose2d(1.8, -0.762, Rotation2d.fromDegrees(90)),
                            new Pose2d(2.6, 0.0, Rotation2d.fromDegrees(0)),
                            new Pose2d(4.9, 0.0, Rotation2d.fromDegrees(0)),
                            new Pose2d(5.6, 0.762, Rotation2d.fromDegrees(90)),
                            new Pose2d(4.9, 1.5, Rotation2d.fromDegrees(180)),
                            new Pose2d(3.9, 0.762, Rotation2d.fromDegrees(-90)),
                            new Pose2d(5.4, -0.762, Rotation2d.fromDegrees(-90)),
                            new Pose2d(6.44, -1.5, Rotation2d.fromDegrees(0)),
                            new Pose2d(7.13, -0.762, Rotation2d.fromDegrees(90)),
                            new Pose2d(6.44, 0.0, Rotation2d.fromDegrees(180)),
                            new Pose2d(-1.0, 0.1, Rotation2d.fromDegrees(180)));

            public static final List<Rotation2d> headings =
                    Arrays.asList(
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0));
            //                            Rotation2d.fromDegrees(0),
            //                            Rotation2d.fromDegrees(0),
            //                            Rotation2d.fromDegrees(-90),
            //                            Rotation2d.fromDegrees(180),
            //                            Rotation2d.fromDegrees(90),
            //                            Rotation2d.fromDegrees(0),
            //                            Rotation2d.fromDegrees(0),
            //                            Rotation2d.fromDegrees(90),
            //                            Rotation2d.fromDegrees(180),
            //                            Rotation2d.fromDegrees(-90),
            //                            Rotation2d.fromDegrees(-90),
            //                            Rotation2d.fromDegrees(0),
            //                            Rotation2d.fromDegrees(90),
            //                            Rotation2d.fromDegrees(180),
            //                            Rotation2d.fromDegrees(180));
        }

        public static class test {
            public static final List<Pose2d> waypoints =
                    Arrays.asList(
                            new Pose2d(0, 0, Rotation2d.fromDegrees(-35)),
                            new Pose2d(1.87, -1.05, Rotation2d.fromDegrees(-35)),
                            new Pose2d(3.05, -2.133, Rotation2d.fromDegrees(-35)),
                            new Pose2d(4.58, -1.0, Rotation2d.fromDegrees(45)),
                            new Pose2d(8.0, 0.0, Rotation2d.fromDegrees(0)));
            public static final List<Rotation2d> headings =
                    Arrays.asList(
                            Rotation2d.fromDegrees(180),
                            Rotation2d.fromDegrees(180),
                            Rotation2d.fromDegrees(180),
                            Rotation2d.fromDegrees(-135),
                            Rotation2d.fromDegrees(-135));
        }

        public static class ShootPositions {
            public static final List<Pose2d> waypoints =
                    Arrays.asList(
                            new Pose2d(0, 0, new Rotation2d(0)),
                            new Pose2d(-4.1, 0, new Rotation2d(0)));
            public static final List<Rotation2d> headings =
                    Arrays.asList(new Rotation2d(0), new Rotation2d(0));

            public static final List<Pose2d> waypoints1 =
                    Arrays.asList(
                            new Pose2d(-4.1, 0, new Rotation2d(0)),
                            new Pose2d(0.0, 0, new Rotation2d(0)));
            public static final List<Rotation2d> headings1 =
                    Arrays.asList(new Rotation2d(0), new Rotation2d(0));
        }
    }
}
