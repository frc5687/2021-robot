/* (C)2020-2021 */
package org.frc5687.infiniterecharge.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.IOException;
import java.nio.file.Path;
import org.frc5687.infiniterecharge.robot.commands.*;
import org.frc5687.infiniterecharge.robot.subsystems.*;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;
import org.frc5687.lib.T265Camera;

public class RobotContainer extends OutliersContainer {

    private OI _oi;
    private AHRS _imu;
    private T265Camera _slamCamera;

    private Robot _robot;
    private DriveTrain _driveTrain;
    private Intake _intake;
    private Spindexer _spindexer;
    private Hood _hood;
    private Shooter _shooter;

    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    public void init() {
        int counter = 0;
        _oi = new OI();
        _imu = new AHRS(SPI.Port.kMXP, (byte) 200);
        _slamCamera = null;

        _driveTrain = new DriveTrain(this, _oi, _imu, _slamCamera);
        _intake = new Intake(this);
        _spindexer = new Spindexer(this);
        _hood = new Hood(this);
        _shooter = new Shooter(this);

        while (++counter <= 1 && _slamCamera == null) {
            try {
                _slamCamera =
                        new T265Camera(
                                Constants.DriveTrain.SLAM_TO_ROBOT,
                                Constants.DriveTrain.T265_MEASUREMENT_COVARIANCE);
                metric("Slam Camera Status", "Working");
            } catch (T265Camera.CameraJNIException | UnsatisfiedLinkError e) {
                _slamCamera = null;
                error("T265Camera not found");
                error(e.getMessage());
                metric("Slam Camera Status", "Broken!");
            }
        }

        String trajectoryJSON = "output/BarrelRace.wpilib.json";
        new Trajectory();
        Trajectory trajectoryNew = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            Transform2d transform =
                    new Pose2d(0, 0, _driveTrain.getHeading()).minus(trajectory.getInitialPose());
            trajectoryNew = trajectory.transformBy(transform);

            error("Trajectory successfully opened.");
        } catch (IOException ex) {
            error("Unable to open trajectory: " + trajectoryJSON + ex.getMessage());
        }
        error("TrajectoryNew staring pose is " + trajectoryNew.getInitialPose().toString());
        _oi.initializeButtons(_driveTrain, _intake, _spindexer, trajectoryNew);
        //        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));
        setDefaultCommand(_intake, new IdleIntake(_intake));
        setDefaultCommand(_spindexer, new IdleSpindexer(_spindexer));
        setDefaultCommand(_hood, new IdleHood(_hood, _oi));
        setDefaultCommand(_shooter, new IdleShooter(_shooter));

        _robot.addPeriodic(this::controllerPeriodic, 0.005, 0.005);
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
        _driveTrain.updateDashboard();
    }

    public void controllerPeriodic() {
        if (_driveTrain != null) {
            _driveTrain.controllerPeriodic();
        }
    }
}
