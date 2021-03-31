/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import java.util.List;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import org.frc5687.infiniterecharge.robot.util.SwerveTrajectory;
import org.frc5687.infiniterecharge.robot.util.SwerveTrajectoryGenerator;

public class DriveTrajectory extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private List<Pose2d> _waypoints;
    private List<Rotation2d> _heading;
    private double _time;
    private boolean _realtime;
    private SwerveTrajectory _trajectory;
    private Trajectory _trajectoryW;
    private final Timer _timer;

    public DriveTrajectory(DriveTrain driveTrain) {
        addRequirements(driveTrain);
        _driveTrain = driveTrain;
        _timer = new Timer();
        enableMetrics();
        logMetrics("x", "y", "heading");
    }

    public DriveTrajectory(DriveTrain driveTrain, Trajectory trajectory) {
        this(driveTrain);
        _trajectoryW = trajectory;
        _realtime = false;
    }

    public DriveTrajectory(DriveTrain driveTrain, SwerveTrajectory swerveTrajectory) {
        this(driveTrain);

        _trajectory = swerveTrajectory;
        _realtime = false;
    }

    public DriveTrajectory(
            DriveTrain driveTrain, List<Pose2d> waypoints, List<Rotation2d> heading) {
        this(driveTrain);
        _waypoints = waypoints;
        _heading = heading;
        _realtime = true;
    }

    @Override
    public void initialize() {
        if (_realtime) {
            _trajectory =
                    SwerveTrajectoryGenerator.generateTrajectory(
                            _waypoints, _heading, _driveTrain.getConfig());
        }
        _time = _trajectory.getTotalTimeSeconds();
        _timer.reset();
        _timer.start();
    }

    @Override
    public void execute() {
        SwerveTrajectory.State goal = _trajectory.sample(_timer.get());
        _driveTrain.setField(goal.poseMeters, goal.heading);

        //        metric("heading", goal.heading.getDegrees());
        //        metric("x", goal.poseMeters.getX());
        //        metric("y", goal.poseMeters.getY());
        _driveTrain.trajectoryFollower(goal);
    }

    @Override
    public boolean isFinished() {
        return _timer.get() == _time;
        //        return (Math.abs(_timer.get() - _time) < 0.1);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _timer.reset();
    }
}
