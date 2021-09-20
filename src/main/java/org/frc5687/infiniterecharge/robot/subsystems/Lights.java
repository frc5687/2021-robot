package org.frc5687.infiniterecharge.robot.subsystems;

import org.frc5687.infiniterecharge.robot.util.MetricTracker;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Lights {
    private AddressableLEDBuffer blinken; 
    private AddressableLED blinkenAddress;
    private MetricTracker metric;

    public Lights(){
        for (var i = 0; i < blinken.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            blinken.setRGB(i, 255, 0, 0);
         }
         blinkenAddress.setData(blinken);
    }

    public void setLights(){
        for (var i = 0; i < blinken.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            blinken.setRGB(i, 255, 255, 255);
         }
         blinkenAddress.setData(blinken);
    }

    public void COPS(){
        for (var i = 0; i < blinken.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            blinken.setRGB(i, 255, 0, 0); //Red
         }
         blinkenAddress.setData(blinken);

         for (var i = 0; i < blinken.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            blinken.setRGB(i, 0, 0, 255); //Blue
         }
         blinkenAddress.setData(blinken);
    }

    
    public void Promo(){
        for (var i = 0; i < blinken.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            blinken.setRGB(i, 255, 215, 0);
         }
         blinkenAddress.setData(blinken);

         for (var i = 0; i < blinken.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            blinken.setRGB(i, 255, 0, 0);
         }
         blinkenAddress.setData(blinken);
    }

    public void green(){
        for (var i = 0; i < blinken.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            blinken.setRGB(i, 255, 0, 0);
         }
         blinkenAddress.setData(blinken);
    }
}
