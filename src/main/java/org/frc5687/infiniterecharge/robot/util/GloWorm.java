/* (C)2021 */
package org.frc5687.infiniterecharge.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import org.photonvision.PhotonCamera;

public class GloWorm {

    private final PhotonCamera _camera;
    private final NetworkTable _table;
    private final NetworkTableEntry _pose;
    private final double[] _defaultVal = {0, 0, 0};

    public GloWorm(String camName) {
        _camera = new PhotonCamera(camName);
        _table = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable(camName);
        _pose = _table.getEntry("targetPose");
    }

    // Latency in milliseconds.
    public double getLatency() {
        return _camera.getLatestResult().getLatencyMillis();
    }

    public double getTargetYaw() {
        return _camera.getLatestResult().getBestTarget().getYaw();
    }

    // Should return pose from 3D mode. Not sure, not documented yet.
    public Pose2d getTargetPose() {
        double[] pose = _table.getEntry("targetPose").getDoubleArray(_defaultVal);
        return new Pose2d(new Translation2d(pose[0], pose[1]), Rotation2d.fromDegrees(pose[2]));
    }

    public boolean hasTarget() {
        return _camera.getLatestResult().hasTargets();
    }
}
