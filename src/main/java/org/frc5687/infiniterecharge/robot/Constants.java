/* Team 5687 (C)2020-2021 */
package org.frc5687.infiniterecharge.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.numbers.*;

public class Constants {

    public static final int TICKS_PER_UPDATE = 1;
    public static final double METRIC_FLUSH_PERIOD = 1.0;
    public static final double UPDATE_PERIOD = 0.02;
    public static final double EPSILON = 0.001;

    public static class DriveTrain {

        public static final double WIDTH = 0.6223;
        public static final double LENGTH = 0.6223;
        public static final Translation2d FRONT_LEFT_POSITION =
                new Translation2d(WIDTH / 2.0, LENGTH / 2.0);
        public static final double FRONT_LEFT_ENCODER_OFFSET = -0.071; // radians
        public static final Translation2d FRONT_RIGHT_POSITION =
                new Translation2d(WIDTH / 2.0, -LENGTH / 2.0);
        public static final double FRONT_RIGHT_ENCODER_OFFSET = -1.01; // radians
        public static final Translation2d BACK_LEFT_POSITION =
                new Translation2d(-WIDTH / 2.0, LENGTH / 2.0);
        public static final double BACK_RIGHT_ENCODER_OFFSET = -0.122; // radians
        public static final Translation2d BACK_RIGHT_POSITION =
                new Translation2d(-WIDTH / 2.0, -LENGTH / 2.0);
        public static final double BACK_LEFT_ENCODER_OFFSET = 0.585 + Math.PI; // radians

        public static final double DEADBAND = 0.1;
        public static final double SENSITIVITY_VX = 0.9;
        public static final double SENSITIVITY_VY = 0.9;
        public static final double SENSITIVITY_OMEGA = 0.3;
        public static final double MAX_MPS = 2.5816;

        //        public static final double MAX_MPS = 1.0;
        public static final double MAX_ANG_VEL = Math.PI * 2.0;
        public static final double MAX_MPSS = 0.5; // accel

        public static final double ANGLE_kP = 3.5;
        public static final double ANGLE_kI = 0.0;
        public static final double ANGLE_kD = 0.0;

        public static final double kP = 10.5;
        public static final double kI = 0.0;
        public static final double kD = 0.5;
        public static final double PROFILE_CONSTRAINT_VEL = 3.0 * Math.PI;
        public static final double PROFILE_CONSTRAINT_ACCEL = Math.PI;
    }

    public static class DifferentialSwerveModule {

        // update rate of our modules 5ms.
        public static final double kDt = 0.005;

        public static final double FALCON_FREE_SPEED =
                Units.rotationsPerMinuteToRadiansPerSecond(6380);
        public static final int TIMEOUT = 200;
        public static final double GEAR_RATIO_WHEEL = 6.46875;
        public static final double GEAR_RATIO_STEER = 9.2;
        public static final double FALCON_RATE = 600.0;
        public static final double WHEEL_RADIUS = 0.0429; // Meters with compression.
        public static final double TICKS_TO_ROTATIONS = 2048.0;
        public static final double VOLTAGE = 12.0;
        public static final double FEED_FORWARD = VOLTAGE / (FALCON_FREE_SPEED / GEAR_RATIO_WHEEL);

        public static final boolean ENABLE_CURRENT_LIMIT = true;
        public static final double CURRENT_LIMIT = 30.0;
        public static final double CURRENT_THRESHOLD = 30.0;
        public static final double CURRENT_TRIGGER_TIME = 0.0;

        // Create Parameters for DiffSwerve State Space
        public static final double INERTIA_WHEEL = 0.005;
        public static final double INERTIA_STEER = 0.004;
        // A weight for how aggressive each state should be ie. 0.08 radians will try to control the
        // angle more aggressively than the wheel angular velocity.
        public static final double Q_AZIMUTH_ANG_VELOCITY = 1.1; // radians per sec
        public static final double Q_AZIMUTH = 0.08; // radians
        public static final double Q_WHEEL_ANG_VELOCITY = 5; // radians per sec
        // This is for Kalman filter which isn't used for azimuth angle due to angle wrapping.
        // Model noise are assuming that our model isn't as accurate as our sensors.
        public static final double MODEL_AZIMUTH_ANGLE_NOISE = .1; // radians
        public static final double MODEL_AZIMUTH_ANG_VELOCITY_NOISE = 5.0; // radians per sec
        public static final double MODEL_WHEEL_ANG_VELOCITY_NOISE = 5.0; // radians per sec
        // Noise from sensors. Falcon With Gearbox causes us to have more uncertainty so we increase
        // the noise.
        public static final double SENSOR_AZIMUTH_ANGLE_NOISE = 0.01; // radians
        public static final double SENSOR_AZIMUTH_ANG_VELOCITY_NOISE = 0.1; // radians per sec
        public static final double SENSOR_WHEEL_ANG_VELOCITY_NOISE = 0.1; // radians per sec
        public static final double CONTROL_EFFORT = VOLTAGE;
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
}
