/* (C)2021 */
package org.frc5687.infiniterecharge.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import org.photonvision.PhotonCamera;

public class GloWorm {

    private final PhotonCamera _camera;
    private final NetworkTable _table;

    public GloWorm(String camName) {
        _camera = new PhotonCamera(camName);
        _table = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable(camName);
    }

    // Latency in milliseconds.
    public double getLatency() {
        return _camera.getLatestResult().getLatencyMillis();
    }

    public double getTargetYaw() {
        return _camera.getLatestResult().getBestTarget().getYaw();
    }

    // Should return pose from 3D mode. Not sure, not documented yet.
    public Transform2d cameraToTarget() {
        return _camera.getLatestResult().getBestTarget().getCameraToTarget();
    }

    public boolean hasTarget() {
        return _camera.getLatestResult().hasTargets();
    }
}
