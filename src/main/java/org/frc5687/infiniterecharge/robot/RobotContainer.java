/* (C)5687-2021 */
package org.frc5687.infiniterecharge.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.infiniterecharge.robot.commands.*;
import org.frc5687.infiniterecharge.robot.commands.auto.StealBallAuto;
import org.frc5687.infiniterecharge.robot.subsystems.*;
import org.frc5687.infiniterecharge.robot.util.AutoChooser;
import org.frc5687.infiniterecharge.robot.util.JetsonProxy;
import org.frc5687.infiniterecharge.robot.util.Limelight;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class RobotContainer extends OutliersContainer {

    private OI _oi;
    private AHRS _imu;
    private JetsonProxy _proxy;
    private AutoChooser _autoChooser;

    private Robot _robot;
    private DriveTrain _driveTrain;
    private Intake _intake;
    private Spindexer _spindexer;
    private Hood _hood;
    private Shooter _shooter;
    private Climber _climber;
    private Limelight _limelight;


    private int count = 0;

    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    public void init() {
        _oi = new OI();
        _imu = new AHRS(SPI.Port.kMXP, (byte) 200);
        _autoChooser = new AutoChooser(getIdentityMode());
        _limelight = new Limelight("limelight");

        //        _intake = new Intake(this);
        //        _hood = new Hood(this);
        //        _spindexer = new Spindexer(this);
        //        _shooter = new Shooter(this);
        //        _climber = new Climber(this);
        //        _driveTrain = new DriveTrain(this, _limelight, _oi, _imu);

        //        setDefaultCommand(_intake, new IdleIntake(_intake));
        //        setDefaultCommand(_spindexer, new IdleSpindexer(_spindexer));
        //        setDefaultCommand(_hood, new IdleHood(_hood, _oi));
        //        setDefaultCommand(_shooter, new IdleShooter(_shooter, _oi));
        //        setDefaultCommand(_climber, new IdleClimber(_climber, _oi));
        //        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));

        Trajectory eightBall = getTrajectory("output/EightBall.wpilib.json");
        Trajectory stealTenBallPrt1 = getTrajectory("output/TenBallPrt1.wpilib.json");
        Trajectory stealTenBallPrt2 = getTrajectory("output/TenBallPrt2.wpilib.json");
        Trajectory stealTenBallPrt3  = getTrajectory("output/TenBallPrt3.wpilib.json");
        Trajectory stealTenBallPrt4  = getTrajectory("output/TenBallPrt4.wpilib.json");
        Trajectory exitTrench = getTrajectory("output/ExitTrench.wpilib.json");

        Field2d field = new Field2d();
        SmartDashboard.putData(field);
        SmartDashboard.putString("test", "test");
        field.getObject("traj").setTrajectory(stealTenBallPrt1);
        //        _oi.initializeButtons(
        //                _driveTrain, _shooter, _intake, _spindexer, _hood, _climber,
        // trajectoryNew);

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

    public Command getAutonomousCommand() {
        AutoChooser.Mode autoMode = _autoChooser.getSelectedMode();

        switch (autoMode) {
            case StealTenBall:
                return wrapCommand(new StealBallAuto(
                        _driveTrain,
                        _shooter,
                        _hood,
                        _intake,
                        _spindexer,
                        getTrajectory("output/TenBallPrt1.wpilib.json"),
                        getTrajectory("output/TenBallPrt2.wpilib.json")
                ));
            default:
                return new ZeroHood(_hood);
        }
    }

    private Trajectory getTrajectory(String trajectoryJSON) {
        Trajectory trajectory = null;
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            error("Trajectory init pose is " + trajectory.getInitialPose().toString());
            error("Trajectory successfully opened.");
        } catch (IOException ex) {
            error("Unable to open trajectory: " + trajectoryJSON + ex.getMessage());
        }
        return trajectory;
    }

    private Command wrapCommand(Command command) {
        return new SequentialCommandGroup(
                new ZeroHood(_hood),
                command
        );
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
