/* (C)2020-2021 */
package org.frc5687.infiniterecharge.robot;

import static org.frc5687.infiniterecharge.robot.Constants.DriveTrain.DEADBAND;
import static org.frc5687.infiniterecharge.robot.util.Helpers.applyDeadband;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.frc5687.infiniterecharge.robot.commands.AutoIntake;
import org.frc5687.infiniterecharge.robot.commands.AutoShoot;
import org.frc5687.infiniterecharge.robot.commands.AutoShootSetpoint;
import org.frc5687.infiniterecharge.robot.commands.DriveTrajectory;
import org.frc5687.infiniterecharge.robot.subsystems.*;
import org.frc5687.infiniterecharge.robot.util.AxisButton;
import org.frc5687.infiniterecharge.robot.util.Gamepad;
import org.frc5687.infiniterecharge.robot.util.OutliersProxy;
import org.frc5687.infiniterecharge.robot.util.POV;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected Joystick _leftJoystick;
    protected Joystick _rightJoystick;
    protected Button _driverRightStickButton;

    private JoystickButton _trigger;
    private JoystickButton _thumbButton;
    private JoystickButton _shootButton;
    private JoystickButton _resetYawButton;

    private Button _driverAButton;
    private Button _driverBButton;
    private Button _driverXButton;
    private Button _driverYButton;
    private Button _driverRightTrigger;

    public OI() {
        _driverGamepad = new Gamepad(0);

        _leftJoystick = new Joystick(1);
        _rightJoystick = new Joystick(2);

        _driverRightStickButton =
                new JoystickButton(_driverGamepad, Gamepad.Buttons.RIGHT_STICK.getNumber());

        _trigger = new JoystickButton(_rightJoystick, 1);
        _thumbButton = new JoystickButton(_rightJoystick, 2);
        _shootButton = new JoystickButton(_leftJoystick, 1);
        _resetYawButton = new JoystickButton(_rightJoystick, 4);

        _driverAButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.A.getNumber());
        _driverBButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.B.getNumber());
        _driverYButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.Y.getNumber());
        _driverXButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.X.getNumber());
        _driverRightTrigger =
                new AxisButton(_driverGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(), 0.2);
    }

    public void initializeButtons(
            DriveTrain driveTrain,
            Intake intake,
            Spindexer spindexer,
            Shooter shooter,
            Hood hood,
            Trajectory trajectory) {
        //        ArrayList<Translation2d> waypoints = new ArrayList<>();
        //        ArrayList<Rotation2d> heading = new ArrayList<>();
        //        waypoints.add(new Translation2d(1, 1));
        //        heading.add(Rotation2d.fromDegrees(90));
        //        _driverAButton.whenPressed(
        //                new DriveTrajectory(
        //                        driveTrain,
        //                        driveTrain.getOdometryPose(),
        //                        waypoints,
        //                        heading,
        //                        new Pose2d(2, 0, new Rotation2d(0))));
        _driverYButton.whenHeld(new AutoShootSetpoint(shooter, spindexer, hood, 3000, 55));
        //        _driverAButton.whenPressed(new AutoHoodSetpoint(hood, 45));
        _trigger.whileHeld(new AutoIntake(intake));
        _shootButton.whileHeld(new AutoShoot(spindexer, shooter));
        //                _driverAButton.whenPressed(new AutoHoodSetpoint(hood, 45));
        _driverBButton.whenPressed(new DriveTrajectory(driveTrain, trajectory));
        _resetYawButton.whenPressed(driveTrain::resetYaw);
    }

    public double getDriveY() {
        double speed = getSpeedFromAxis(_leftJoystick, _leftJoystick.getYChannel());
        //        double speed = getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        speed = applyDeadband(speed, DEADBAND);
        return speed;
    }

    public double getDriveX() {
        double speed = -getSpeedFromAxis(_leftJoystick, _leftJoystick.getXChannel());
        //        double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_X.getNumber());
        speed = applyDeadband(speed, DEADBAND);
        return speed;
    }

    public double getRotationX() {
        double speed = getSpeedFromAxis(_rightJoystick, _rightJoystick.getXChannel());
        //        double speed = getSpeedFromAxis(_driverGamepad, Gamepad.Axes.RIGHT_X.getNumber());
        speed = applyDeadband(speed, DEADBAND);
        return speed;
    }

    public double getHoodSpeed() {
        double speed = getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        speed = applyDeadband(speed, DEADBAND);
        return speed;
    }

    public double getFlywheelSpeed() {
        double speed = getSpeedFromAxis(_driverGamepad, Gamepad.Axes.RIGHT_Y.getNumber());
        speed = applyDeadband(speed, DEADBAND);
        return speed;
    }

    public int getDriverPOV() {
        return POV.fromWPILIbAngle(0, _driverGamepad.getPOV()).getDirectionValue();
    }

    public boolean holdAngle() {
        return _thumbButton.get();
    }

    protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

    @Override
    public void updateDashboard() {}
}
