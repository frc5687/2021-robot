/* (C)5687-2021 */
package org.frc5687.infiniterecharge.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5687.infiniterecharge.robot.commands.*;
import org.frc5687.infiniterecharge.robot.subsystems.*;
import org.frc5687.infiniterecharge.robot.util.JetsonProxy;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class RobotContainer extends OutliersContainer {

    private OI _oi;
    private AHRS _imu;

    private Robot _robot;
    private DriveTrain _driveTrain;
    private Intake _intake;
    private Spindexer _spindexer;
    private Hood _hood;
    private Shooter _shooter;
    private Climber _climber;
    private JetsonProxy _proxy;

    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    public void init() {
        _oi = new OI();
        _imu = new AHRS(SPI.Port.kMXP, (byte) 200);
        _proxy = new JetsonProxy(10);

        //        _intake = new Intake(this);
        //        _hood = new Hood(this);
        //        _spindexer = new Spindexer(this);
        //        _shooter = new Shooter(this);
        //        _climber = new Climber(this);
        _driveTrain = new DriveTrain(this, _proxy, _oi, _imu);

        //        setDefaultCommand(_intake, new IdleIntake(_intake));
        //        setDefaultCommand(_spindexer, new IdleSpindexer(_spindexer));
        //        setDefaultCommand(_hood, new IdleHood(_hood, _oi));
        //        setDefaultCommand(_shooter, new IdleShooter(_shooter, _oi));
        //        setDefaultCommand(_climber, new IdleClimber(_climber, _oi));
        //        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));
        //        _oi.initializeButtons(_driveTrain, _shooter, _intake, _spindexer, _hood,
        // _climber);

        _robot.addPeriodic(this::controllerPeriodic, 0.010, 0.005);

        _imu.reset();
    }

    public void periodic() {}

    public void disabledPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void teleopInit() {}

    @Override
    public void autonomousInit() {}

    private void setDefaultCommand(OutliersSubsystem subSystem, OutliersCommand command) {
        if (subSystem == null || command == null) {
            return;
        }
        CommandScheduler s = CommandScheduler.getInstance();
        s.setDefaultCommand(subSystem, command);
    }

    @Override
    public void updateDashboard() {
        super.updateDashboard();
    }

    public void controllerPeriodic() {
        if (_driveTrain != null) {
            _driveTrain.controllerPeriodic();
        }
    }
}
