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
    private final boolean _realtime;
    private SwerveTrajectory _trajectory;
    private Trajectory _trajectoryW;
    private final Timer _timer;

    public DriveTrajectory(DriveTrain driveTrain, Trajectory trajectory) {
        addRequirements(driveTrain);
        _driveTrain = driveTrain;
        _trajectoryW = trajectory;
        _realtime = false;
        _timer = new Timer();
    }

    public DriveTrajectory(
            DriveTrain driveTrain, List<Pose2d> waypoints, List<Rotation2d> heading) {
        addRequirements(driveTrain);
        _driveTrain = driveTrain;
        _waypoints = waypoints;
        _heading = heading;
        _realtime = true;
        _timer = new Timer();
    }

    @Override
    public void initialize() {
        if (_realtime) {
            _trajectory =
                    SwerveTrajectoryGenerator.generateTrajectory(
                            _waypoints, _heading, _driveTrain.getConfig());
        }
        _time = _trajectoryW.getTotalTimeSeconds();
        _timer.reset();
        _timer.start();
    }

    @Override
    public void execute() {
        Trajectory.State goal = _trajectoryW.sample(_timer.get());
        _driveTrain.trajectoryFollower(goal, new Rotation2d(0));
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
