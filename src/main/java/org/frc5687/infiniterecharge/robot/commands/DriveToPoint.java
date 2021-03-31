/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import org.frc5687.infiniterecharge.robot.util.SwerveTrajectory;

public class DriveToPoint extends OutliersCommand {

    private DriveTrain _driveTrain;
    private SwerveTrajectory.State _goal;

    public DriveToPoint(DriveTrain driveTrain, Pose2d goalPose) {
        _driveTrain = driveTrain;
        _goal = new SwerveTrajectory.State(0, 0, 0, goalPose, 0, new Rotation2d(0));
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        _driveTrain.trajectoryFollower(_goal);
    }

    @Override
    public boolean isFinished() {
        return _goal.poseMeters.equals(_driveTrain.getOdometryPose());
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
