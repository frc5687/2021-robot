/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;

public class DriveToPose extends OutliersCommand {

    private final DriveTrain _drivetrain;
    private final Pose2d _pose;
    private final Rotation2d _heading;
    private final double _vel;

    public DriveToPose(DriveTrain drivetrain, Pose2d pose, Rotation2d heading, double vel) {
        _drivetrain = drivetrain;
        _pose = pose;
        _heading = heading;
        _vel = vel;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        super.execute();
        _drivetrain.poseFollower(_pose, _heading, _vel);
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
