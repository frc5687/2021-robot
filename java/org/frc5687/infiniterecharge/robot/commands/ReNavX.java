package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;

public class ReNavX extends OutliersCommand{
    private DriveTrain driveTrain;
    public ReNavX(DriveTrain _driveTrain){
        driveTrain = _driveTrain;
    }

    public void initialize(){
        super.initialize();
        super.execute();
        driveTrain.resetNavX();
        System.out.print("NavX reset");
    }

    public void execute(){
        super.execute();
    }
}