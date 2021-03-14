/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import java.util.ArrayList;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;

public class DriveTrajectory extends OutliersCommand {

    private DriveTrain _driveTrain;
    private Pose2d _start;
    private Pose2d _end;
    private ArrayList<Translation2d> _waypoints;
    private ArrayList<Rotation2d> _heading;
    private double _time;
    private final boolean _realtime;
    private Trajectory _trajectory;
    private Timer _timer;

    public DriveTrajectory(DriveTrain driveTrain, Trajectory trajectory) {
        addRequirements(driveTrain);
        _driveTrain = driveTrain;
        _trajectory = trajectory;
        _realtime = false;
        _timer = new Timer();
    }

    public DriveTrajectory(
            DriveTrain driveTrain,
            Pose2d start,
            ArrayList<Translation2d> waypoints,
            ArrayList<Rotation2d> heading,
            Pose2d end) {
        addRequirements(driveTrain);
        _driveTrain = driveTrain;
        _start = start;
        _waypoints = waypoints;
        _heading = heading;
        _end = end;
        _realtime = true;
        _timer = new Timer();
        _heading.add(0, _start.getRotation());
        _heading.add(_end.getRotation());
    }

    @Override
    public void initialize() {
        if (_realtime) {
            _trajectory =
                    TrajectoryGenerator.generateTrajectory(
                            _start, _waypoints, _end, _driveTrain.getConfig());
        }
        _time = _trajectory.getTotalTimeSeconds();
        _timer.reset();
        _timer.start();
    }

    @Override
    public void execute() {
        int n = 0;
        Trajectory.State goal = _trajectory.sample(_timer.get());
        if (goal.poseMeters.getTranslation().equals(_waypoints.get(n))) {
            n++;
        }
        _driveTrain.trajectoryFollower(goal, _heading.get(n + 1));
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
