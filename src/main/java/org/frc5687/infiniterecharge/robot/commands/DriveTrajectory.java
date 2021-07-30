/* (C)5687-2021 */
package org.frc5687.infiniterecharge.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;

public class DriveTrajectory extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private Trajectory _trajectory;
    private Rotation2d _heading;
    private final Timer _timer;

    public DriveTrajectory(DriveTrain driveTrain) {
        _driveTrain = driveTrain;
        _timer = new Timer();
        addRequirements(_driveTrain);
    }
    public DriveTrajectory(DriveTrain driveTrain, Trajectory trajectory) {
        this(driveTrain);
        _trajectory = trajectory;
        _heading = new Rotation2d(0);
    }
    public DriveTrajectory(DriveTrain driveTrain, Trajectory trajectory, Rotation2d heading) {
        this(driveTrain, trajectory);
        _heading = heading;
    }

    @Override
    public void initialize() {
        super.initialize();
        _timer.reset();
        _timer.start();
    }

    @Override
    public void execute() {
        super.execute();
        Trajectory.State goal = _trajectory.sample(_timer.get());
        _driveTrain.trajectoryFollower(goal, _heading);
    }

    @Override
    public boolean isFinished() {
        return _timer.get() >= _trajectory.getTotalTimeSeconds();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _timer.reset();
    }
}
