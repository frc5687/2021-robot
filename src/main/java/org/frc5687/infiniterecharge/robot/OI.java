/* (C)2020-2021 */
package org.frc5687.infiniterecharge.robot;

import static org.frc5687.infiniterecharge.robot.util.Helpers.applyDeadband;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.ArrayList;
import org.frc5687.infiniterecharge.robot.commands.DriveTrajectory;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import org.frc5687.infiniterecharge.robot.util.Gamepad;
import org.frc5687.infiniterecharge.robot.util.OutliersProxy;
import org.frc5687.infiniterecharge.robot.util.POV;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected Button _driverRightStickButton;

    private Button _driverAButton;
    private Button _driverBButton;
    private Button _driverXButton;
    private Button _driverYButton;

    public OI() {
        _driverGamepad = new Gamepad(0);

        _driverRightStickButton =
                new JoystickButton(_driverGamepad, Gamepad.Buttons.RIGHT_STICK.getNumber());

        _driverAButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.A.getNumber());
        _driverBButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.B.getNumber());
        _driverYButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.Y.getNumber());
        _driverXButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.X.getNumber());
    }

    public void initializeButtons(DriveTrain driveTrain, Trajectory trajectory) {
        var waypoints = new ArrayList<Translation2d>();
        var angleTest = new ArrayList<Translation2d>();
        waypoints.add(new Translation2d(1, -1));
        //                waypoints.add(new Translation2d(2, 0));
        _driverAButton.whenPressed(
                new DriveTrajectory(
                        driveTrain,
                        driveTrain.getOdometryPose(),
                        waypoints,
                        new Pose2d(0, 0, new Rotation2d(0))));
        _driverBButton.whenPressed(new DriveTrajectory(driveTrain, trajectory));
        _driverXButton.whenPressed(
                new DriveTrajectory(
                        driveTrain,
                        driveTrain.getOdometryPose(),
                        angleTest,
                        new Pose2d(
                                driveTrain.getOdometryPose().getX(),
                                driveTrain.getOdometryPose().getY(),
                                new Rotation2d(Math.PI / 2))));
        //        _driverBButton.whenPressed(null);
    }

    public double getDriveY() {
        double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        speed = applyDeadband(speed, Constants.DriveTrain.DEADBAND);
        return speed;
    }

    public double getDriveX() {
        double speed = getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_X.getNumber());
        speed = applyDeadband(speed, Constants.DriveTrain.DEADBAND);
        return speed;
    }

    public double getRotationX() {
        double speed = getSpeedFromAxis(_driverGamepad, Gamepad.Axes.RIGHT_X.getNumber());
        speed = applyDeadband(speed, Constants.DriveTrain.DEADBAND);
        return speed;
    }

    public int getDriverPOV() {
        return POV.fromWPILIbAngle(0, _driverGamepad.getPOV()).getDirectionValue();
    }

    public boolean holdAngle() {
        return _driverRightStickButton.get();
    }

    protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

    @Override
    public void updateDashboard() {}
}
