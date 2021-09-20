package org.frc5687.infiniterecharge.robot.util;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

public class POS {
    private RamseteController ramsete1 = new RamseteController();
    private RamseteController ramsete2 = new RamseteController();
    private Trajectory trajectory;
    public POS(){

    }

    public double sampleTrij(){
        return 0; //For debugging should remove later
        /*Trajectory.State goal = trajectory.sample(3.4); // sample the trajectory at 3.4 seconds from the beginning
        ChassisSpeeds adjustedSpeeds = ramsete1.calculate(currentRobotPose, goal);*/
    }
}
