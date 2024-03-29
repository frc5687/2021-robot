/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands.shooter;

import javax.sound.sampled.LineEvent;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.commands.OutliersCommand;
import org.frc5687.infiniterecharge.robot.subsystems.Hood;
import org.frc5687.infiniterecharge.robot.subsystems.Shooter;
import org.frc5687.infiniterecharge.robot.subsystems.Spindexer;
import org.frc5687.infiniterecharge.robot.util.Limelight;


public class Shoot extends OutliersCommand {

    private final Shooter _shooter;
    private final Spindexer _spindexer;
    private final Hood _hood;
    private Limelight limeLight;
    private Long _endTime;

    public Shoot(Shooter shooter, Spindexer spindexer, Hood hood) {
        _shooter = shooter;
        _spindexer = spindexer;
        _hood = hood;
        //limeLight.enableLEDS(); //Can cause a crash
        addRequirements(_spindexer);
    }

    @Override
    public void initialize() {
        super.initialize();
        _endTime = null;
    }

    @Override
    public void execute() {
        super.execute();
        //Make sure the shooter is at shooting speed and hold is at the correct height
        if (_shooter.isAtTargetVelocity() && _hood.isAtReference()) {
            _endTime = System.currentTimeMillis() + Constants.Shooter.TIMEOUT;
            //Spin up the spindexer to the shooting shoot
            _spindexer.setSpindexerSpeed(Constants.Spindexer.SPINDEXER_SPEED);
            //Spin up the power cell feeder
            _spindexer.setFeederSpeed(Constants.Spindexer.FEEDER_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        if (_endTime == null) {
            return false;
        }
        return System.currentTimeMillis() >= _endTime;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
