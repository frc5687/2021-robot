/* (C)2021 */
package org.frc5687.infiniterecharge.robot.commands.shooter;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.Hood;
import org.frc5687.infiniterecharge.robot.subsystems.Shooter;
import org.frc5687.infiniterecharge.robot.subsystems.Spindexer;

public class AutoShoot extends Shoot {

    private Spindexer _spindexer;
    private long _delayMillis;
    private long _endMillis = 0;

    public AutoShoot(Shooter shooter, Spindexer spindexer, Hood hood) {
        super(shooter, spindexer, hood);
        _spindexer = spindexer;
    }

    @Override
    public void initialize() {
        _delayMillis = System.currentTimeMillis() + Constants.Shooter.AUTO_SHOOT_DELAY;
    }

    @Override
    public void execute() {
        super.execute();
        if (System.currentTimeMillis() > _delayMillis) {
            if (_endMillis == 0) {
                super.initialize();
                _endMillis = System.currentTimeMillis() + Constants.Shooter.AUTO_SHOOT_RUNON;
            }
            super.execute();
        }
    }

    @Override
    public boolean isFinished() {
        return (_endMillis > 0 && System.currentTimeMillis() > _endMillis);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _spindexer.setFeederSpeed(0);
    }
}
