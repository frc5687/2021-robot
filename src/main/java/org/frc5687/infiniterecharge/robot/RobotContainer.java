/* (C)5687-2021 */
package org.frc5687.infiniterecharge.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.io.IOException;
import java.nio.file.Path;
import org.frc5687.infiniterecharge.robot.commands.*;
import org.frc5687.infiniterecharge.robot.commands.auto.ShootAndGo;
import org.frc5687.infiniterecharge.robot.commands.climber.IdleClimber;
import org.frc5687.infiniterecharge.robot.commands.shooter.IdleHood;
import org.frc5687.infiniterecharge.robot.commands.shooter.IdleShooter;
import org.frc5687.infiniterecharge.robot.commands.shooter.ZeroHood;
import org.frc5687.infiniterecharge.robot.subsystems.*;
import org.frc5687.infiniterecharge.robot.util.JetsonProxy;
import org.frc5687.infiniterecharge.robot.util.Limelight;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class RobotContainer extends OutliersContainer {

    private OI _oi;
    private AHRS _imu;
    private JetsonProxy _proxy;

    private Robot _robot;
    private DriveTrain _driveTrain;
    private Intake _intake;
    private Spindexer _spindexer;
    private Hood _hood;
    private Shooter _shooter;
    private Climber _climber;
    private Limelight _limelight;

    private int count = 0;
    private Trajectory _stealTenPrt1;
    private Trajectory _stealExit;
    private Trajectory _go;

    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    public void init() {
        _oi = new OI();
        _imu = new AHRS(SPI.Port.kMXP, (byte) 200);
        _limelight = new Limelight("limelight");

        _intake = new Intake(this);
        _hood = new Hood(this);
        _spindexer = new Spindexer(this);
        _shooter = new Shooter(this);
        _climber = new Climber(this);
        _driveTrain = new DriveTrain(this, _limelight, _oi, _imu);
        //
        setDefaultCommand(_intake, new IdleIntake(_intake));
        setDefaultCommand(_spindexer, new IdleSpindexer(_spindexer));
        setDefaultCommand(_hood, new IdleHood(_hood, _oi));
        setDefaultCommand(_shooter, new IdleShooter(_shooter, _oi));
        setDefaultCommand(_climber, new IdleClimber(_climber, _oi));
        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));
        //

        Trajectory test = null;
        var config = _driveTrain.getConfig();
        config.setReversed(true);
        test =
                TrajectoryGenerator.generateTrajectory(
                        Constants.AutoPath.EightBallAuto.waypoints, config);

        Trajectory eightBall = getTrajectory("output/EightBall.wpilib.json");
        _stealTenPrt1 = getTrajectory("output/StealPathPrt1.wpilib.json");
        Trajectory stealTenBallPrt2 = getTrajectory("output/StealPathPrt2.wpilib.json");
        Trajectory stealTenBallPrt3 = getTrajectory("output/StealPathPrt3.wpilib.json");
        Trajectory stealTenBallPrt4 = getTrajectory("output/StealPathPrt4.wpilib.json");
        _stealExit = getTrajectory("output/StealExit.wpilib.json");
        _go = getTrajectory("output/Go.wpilib.json");
        Trajectory exitTrench = getTrajectory("output/ExitTrench.wpilib.json");

        //        SmartDashboard.putData(field);
        //        SmartDashboard.putString("test", "test");
        //        field.getObject("traj").setTrajectory(stealTenBallPrt1);
        //        field.getObject("traj1").setTrajectory(stealTenBallPrt2);
        //        field.getObject("traj2").setTrajectory(stealTenBallPrt3);
        _oi.initializeButtons(
                _driveTrain,
                _shooter,
                _intake,
                _spindexer,
                _hood,
                _climber,
                _stealTenPrt1,
                _stealExit,
                stealTenBallPrt3,
                stealTenBallPrt4);
        _robot.addPeriodic(this::controllerPeriodic, 0.005, 0.005);
        _driveTrain.resetOdom(_go.getInitialPose());
        var field = new Field2d();
        SmartDashboard.putData(field);
        field.getObject("traj").setTrajectory(test);
        _imu.reset();
    }

    public void periodic() {
        if (_oi.isKillAllPressed()) {
            new KillAll(_driveTrain, _shooter, _spindexer, _hood, _climber).schedule();
        }
    }

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

    public Command getAutonomousCommand() {
        //        return new StealBallAuto(
        //                _driveTrain, _shooter, _hood, _intake, _spindexer, _stealTenPrt1,
        // _stealExit, _oi);
        return new ShootAndGo(_driveTrain, _shooter, _hood, _spindexer, _go, _oi);
        //        return null;
    }

    private Trajectory getTrajectory(String trajectoryJSON) {
        Trajectory trajectory = null;
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            error("Trajectory init pose is " + trajectory.getInitialPose().toString());
            //            trajectory = trajectory.transformBy(transform);
            error("Trajectory successfully opened.");
        } catch (IOException ex) {
            error("Unable to open trajectory: " + trajectoryJSON + ex.getMessage());
        }
        return trajectory;
    }

    private Command wrapCommand(Command command) {
        return new SequentialCommandGroup(new ZeroHood(_hood), command);
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
