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
        //Create a new camera
        _camera = new PhotonCamera(cameraName);
        //Create a new network table
        _table = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable(cameraName);
    }

    public void disableLEDS(){
        //turns off LEDS
        _camera.setLED(LEDMode.kOff);
    }

    public void setDriveMode(boolean mode){
        //Stop the bright
        _camera.setLED(LEDMode.kOff);
        //Normal camera mode
        _camera.setDriverMode(mode);
    }

    public void snapShotInput(){
        //Takes a screen shot of camera input
        _camera.takeInputSnapshot();
    }

    public void snapShotOutput(){
        //Takes a screen shot of camera output
        _camera.takeOutputSnapshot();
    }

    public void enableLEDS(){
        //turns on LEDS
        _camera.setLED(LEDMode.kOn);
    }

    public void blinkLEDS(){
        //Blink LEDS
        //Not even sure if this works
        _camera.setLED(LEDMode.kBlink);
    }

    public LEDMode getLEDS(){
        //Gets the LED mode
        return _camera.getLEDMode();
    }

    public double getLatency() {
        //Gets the latency of the camera
        return _camera.getLatestResult().getLatencyMillis();
    }

    public double getTargetYaw() {
        //Gets the realitive yaw of the target
        return _camera.getLatestResult().getBestTarget().getYaw();
    }

    public double getArea(){
        //Get area of bounding box of target
        return _camera.getLatestResult().getBestTarget().getArea();
    }

    public double getPitch(){
        //Gets realitive pitch of target
        return _camera.getLatestResult().getBestTarget().getPitch();
    }

    public double getSkew(){
        return _camera.getLatestResult().getBestTarget().getSkew();
    }

    public boolean hasTarget() {
        //Does the limelight have a target
        return _camera.getLatestResult().hasTargets();
    }
}
//Kilroy Was Here