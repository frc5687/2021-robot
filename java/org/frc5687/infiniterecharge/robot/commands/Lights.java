package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.util.MetricTracker;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Lights extends OutliersCommand {
    private AddressableLEDBuffer blinken; 
    private AddressableLED blinkenAddress;
    private MetricTracker metric;
    private String status;

    public Lights(int r, int b, int g){
        for (var i = 0; i < blinken.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            blinken.setRGB(i, r, b, g); //Not a primary color(spelled like Murica for Ian, please add in egale call) Ian!!
         }
         blinkenAddress.setData(blinken);
    }

    public void setLights(){
        for (var i = 0; i < blinken.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            blinken.setRGB(i, 255, 255, 255);
         }
         blinkenAddress.setData(blinken);
         status = "white";
    }

    public void COPS(){
        status = "COPS";
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
        status = "Promo";
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
        status = "green";
        for (var i = 0; i < blinken.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            blinken.setRGB(i, 255, 0, 0);
         }
         blinkenAddress.setData(blinken);
    }

    public void updateStatus(){
        metric.put("Blinken Status", status);
    }
}
