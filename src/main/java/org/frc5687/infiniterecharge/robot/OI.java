/* (C)2020-2021 */
package org.frc5687.infiniterecharge.robot;

import static org.frc5687.infiniterecharge.robot.Constants.DriveTrain.DEADBAND;
import static org.frc5687.infiniterecharge.robot.util.Helpers.applyDeadband;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.frc5687.infiniterecharge.robot.commands.*;
import org.frc5687.infiniterecharge.robot.subsystems.*;
import org.frc5687.infiniterecharge.robot.util.*;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected Joystick _leftJoystick;
    protected Joystick _rightJoystick;
    protected Joystick _singleJoystick;
    protected Joystick _rightJoystickTwist;

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

    private double xIn;
    private double yIn;

    public OI() {
        _driverGamepad = new Gamepad(0);

        _leftJoystick = new Joystick(1);
        _rightJoystick = new Joystick(2);
        _singleJoystick = new Joystick(3);
        _rightJoystickTwist = new Joystick(4);

        _driverRightStickButton =
                new JoystickButton(_driverGamepad, Gamepad.Buttons.RIGHT_STICK.getNumber());

        _trigger = new JoystickButton(_singleJoystick, 1);
        _thumbButton = new JoystickButton(_singleJoystick, 2);
        _shootButton = new JoystickButton(_leftJoystick, 1);
        _resetYawButton = new JoystickButton(_singleJoystick, 4);

        _driverAButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.A.getNumber());
        _driverBButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.B.getNumber());
        _driverYButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.Y.getNumber());
        _driverXButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.X.getNumber());
        _driverRightTrigger =
                new AxisButton(_driverGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(), 0.2);
        xIn = 0;
        yIn = 0;
    }

    public void initializeButtons(
            DriveTrain driveTrain,
            Intake intake,
            Spindexer spindexer,
            Shooter shooter,
            Hood hood,
            SwerveTrajectory trajectory,
            SwerveTrajectory trajectory1) {
        //        _driverAButton.whenPressed(
        //                new DriveTrajectory(
        //                        driveTrain,
        //                        Constants.AutoPaths.test.waypoints,
        //                        Constants.AutoPaths.test.headings));
        _driverAButton.whenPressed(new DriveTrajectory(driveTrain, trajectory));
        //        _driverBButton.whenPressed(new DriveTrajectory(driveTrain, trajectory1));
        //        _driverBButton.whenPressed(new DriveTrajectoryAndIntake(driveTrain, intake,
        // trajectory));
        //                        driveTrain.getOdometryPose(),
        //                        waypoints,
        //                        heading,
        //                        new Pose2d(2, 0, new Rotation2d(0))));
        //        _driverAButton.whenHeld(new AutoShootSetpoint(shooter, spindexer, hood, 3500,
        // 48));
        //        _shootButton.whenHeld(new AutoShootSetpoint(shooter, spindexer, hood, 4100, 70));
        //        _driverYButton.whenHeld(new AutoShootSetpoint(shooter, spindexer, hood, 4500,
        // 72));
        //        _driverXButton.whenHeld(new AutoShootSetpoint(shooter, spindexer, hood, 5000,
        // 78)); // 4th
        //        //        _driverAButton.whenPressed(new AutoHoodSetpoint(hood, 45));
        _trigger.whileHeld(new AutoIntake(intake));
        //        _shootButton.whileHeld(new AutoShoot(spindexer, shooter));
        //                        _driverAButton.whenPressed(new AutoHoodSetpoint(hood, 45));
        _resetYawButton.whenPressed(driveTrain::resetOdometry);
    }

    public double getDriveY() {
        yIn = getSpeedFromAxis(_leftJoystick, _leftJoystick.getYChannel());
        yIn = applyDeadband(yIn, 0.1);
        //        double speed = getSpeedFromAxis(_singleJoystick, _singleJoystick.getYChannel());
        double yOut = yIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + 0.0001);
        yOut = (yOut + (yIn * 2)) / 3.0;
        //        double speed = getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        return yOut;
    }

    public double getDriveX() {
        xIn = -getSpeedFromAxis(_leftJoystick, _leftJoystick.getXChannel());
        xIn = applyDeadband(xIn, 0.1);

        double xOut = xIn / (Math.sqrt(xIn * xIn + (yIn * yIn)) + 0.0001);
        xOut = (xOut + (xIn * 2)) / 3.0;
        //        double speed = -getSpeedFromAxis(_singleJoystick, _singleJoystick.getXChannel());
        //        double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_X.getNumber());
        return xOut;
    }

    public double getRotationX() {
        //                double speed = getSpeedFromAxis(_rightJoystick,
        // _rightJoystick.getXChannel());
        double speed = getSpeedFromAxis(_singleJoystick, _singleJoystick.getZChannel());
        //        double speed = getSpeedFromAxis(_rightJoystickTwist,
        // _rightJoystickTwist.getZChannel());
        //        double speed = getSpeedFromAxis(_driverGamepad, Gamepad.Axes.RIGHT_X.getNumber());
        speed = applyDeadband(speed, 0.2);
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
