package org.frc5687.infiniterecharge.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;

public class DriveToPose extends OutliersCommand {

    private final DriveTrain _drivetrain;
    private final Pose2d _pose;
    private final Rotation2d _heading;

    public DriveToPose(DriveTrain drivetrain, Pose2d pose, Rotation2d heading) {
        _drivetrain = drivetrain;
        _pose = pose;
        _heading = heading;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        super.execute();
        _drivetrain.poseFollower(_pose, _heading);
    }

    @Override
    public boolean isFinished() {
        return _drivetrain.isAtPose(_pose);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
