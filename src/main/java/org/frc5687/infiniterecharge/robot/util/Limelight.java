/* (C)2021 */
package org.frc5687.infiniterecharge.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import org.photonvision.LEDMode;
import org.photonvision.PhotonCamera;

public class Limelight {
    private final PhotonCamera _camera;
    private final NetworkTable _table;
    
    public Limelight(String cameraName) {
        _camera = new PhotonCamera(cameraName);
        _table = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable(cameraName);
    }

    public LEDMode getLEDS(){
        return _camera.getLEDMode();
    }

    public double getLatency() {
        return _camera.getLatestResult().getLatencyMillis();
    }

    public double getTargetYaw() {
        return _camera.getLatestResult().getBestTarget().getYaw();
    }

    public boolean hasTarget() {
        return _camera.getLatestResult().hasTargets();
    }
}
