package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import org.frc5687.infiniterecharge.robot.util.MetricTracker;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;


public class Maverick extends OutliersCommand{
    private Pose2d destnation;
    private DriveTrain driveTrain;
    private MetricTracker Metric;
    

    /**
     * Maverick State-Space Model (which may not be added)
     * Properties
     * States: [Meters Pre Second]
     * Input: [Distance to target]
     * Output: [Velocity]
     * @param _driveTrain
     */

    public Maverick(DriveTrain _driveTrain){
        driveTrain = _driveTrain;
        addRequirements(driveTrain);
    }

    public void Afterburner(){
        //Pushes the drive train as fast as it will safely go hyptheticly 
        Constants.DriveTrain.MAX_MPS = 4.0;
    }

    public boolean getCheckPoints(int x1, int y1, int x2, int y2, int x, int y){
        //Is the point inside of the rectangle 
        //Double check the math
        if(x > x2 && x < x2 && y > y1 && y > y2){
            //Inside of the rectangle
            System.out.println("Maverick: " + true);
            //блин!!
            return true;
        }
        else{
            //Not inside of the rectangle
            System.out.println("Maverick: " + false);
            return false;
        }
    }

    public void genLinearSystem(){

    }

    public void wayPointMove(){
        //Iterate through all of the waypoints
        metric("MAVERICK", "Running");
        for(int i = 0; i < Constants.Maverick.numberOfWaypoints; i++){
            metric("MAVERICK", "At waypoint: " + i);
            //Create translations and rotations based off of the Maverick presets
            Translation2d move = new Translation2d(Constants.Maverick.waypointsX[i], Constants.Maverick.waypointsY[i]);
            Rotation2d rotation = new Rotation2d(Constants.Maverick.rotations[i]);
            destnation = new Pose2d(move, rotation);
            //Update the speeds with the realivent Maverick speed
            Constants.DriveTrain.MAX_MPS = Constants.Maverick.speeds[i];
            //Move the robot
            driveTrain.poseFollower(destnation, rotation, 3.5);
        }
        metric("MAVERICK", "Move(s) Complete");
    }

    @Override
    public void initialize() {}

    @Override public void execute(){
        super.execute();
        metric("MAVERICK", "Executeing");
        wayPointMove();
    }

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
    }

    
    @Override
    public boolean isFinished(){
        //Is the robot at it's end position
        metric("MAVERICK", "Finished");
        return driveTrain.MaverickDone(destnation);
    }
}
