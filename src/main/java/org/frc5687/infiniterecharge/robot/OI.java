/* (C)5687-2021 */
package org.frc5687.infiniterecharge.robot;

import static org.frc5687.infiniterecharge.robot.Constants.DriveTrain.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.function.BooleanSupplier;
import org.frc5687.infiniterecharge.robot.commands.*;
import org.frc5687.infiniterecharge.robot.subsystems.*;
import org.frc5687.infiniterecharge.robot.util.AxisButton;
import org.frc5687.infiniterecharge.robot.util.Gamepad;
import org.frc5687.infiniterecharge.robot.util.Helpers;
import org.frc5687.infiniterecharge.robot.util.OutliersProxy;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected Gamepad _operatorGamepad;
    protected Joystick _leftJoystick;
    protected Joystick _rightJoystick;

    protected Button _driverRightStickButton;

    private final JoystickButton _thumbButton;
    private final JoystickButton _shootButton;
    private final JoystickButton _resetYawButton;

    private final Button _driverAButton;
    private final Button _driverBButton;
    private final Button _driverXButton;
    private final Button _driverYButton;
    private final Button _driverRightTrigger;

    private final Button _operatorAButton;
    private final Button _operatorBButton;
    private final Button _operatorXButton;
    private final Button _operatorYButton;

    private final Button _operatorRightTrigger;
    private final Button _operatorLeftTrigger;

    private double yIn = 0;
    private double xIn = 0;

    public OI() {
        _driverGamepad = new Gamepad(0);
        _operatorGamepad = new Gamepad(3);

        _leftJoystick = new Joystick(1);
        _rightJoystick = new Joystick(2);

        _driverRightStickButton =
                new JoystickButton(_driverGamepad, Gamepad.Buttons.RIGHT_STICK.getNumber());

        _thumbButton = new JoystickButton(_rightJoystick, 2);
        _shootButton = new JoystickButton(_leftJoystick, 1);
        _resetYawButton = new JoystickButton(_rightJoystick, 4);

        _driverAButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.A.getNumber());
        _driverBButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.B.getNumber());
        _driverYButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.Y.getNumber());
        _driverXButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.X.getNumber());

        _driverRightTrigger =
                new AxisButton(_driverGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(), 0.2);

        _operatorAButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.A.getNumber());
        _operatorBButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.B.getNumber());
        _operatorXButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.X.getNumber());
        _operatorYButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.Y.getNumber());

        _operatorRightTrigger =
                new AxisButton(_operatorGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(), 0.2);
        _operatorLeftTrigger =
                new AxisButton(_operatorGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber(), 0.2);
    }

    public void initializeButtons(
            DriveTrain drivetrain,
            Shooter shooter,
            Intake intake,
            Spindexer spindexer,
            Hood hood,
            Climber climber) {
        BooleanSupplier limit = () -> DriverStation.getInstance().getMatchNumber() <= 30.0;
        _operatorYButton.whenPressed(
                new ConditionalCommand(new Climb(climber), new IdleClimber(climber, this), limit));
        _operatorRightTrigger.whileHeld(new Shoot(shooter, spindexer));
        _operatorLeftTrigger.whileHeld(new AutoIntake(intake));
        _driverRightTrigger.whileHeld(new AutoTarget(drivetrain, shooter, hood));
    }

    public double getDriveY() {
        yIn = getSpeedFromAxis(_leftJoystick, _leftJoystick.getYChannel());
        //        yIn = getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        yIn = Helpers.applyDeadband(yIn, DEADBAND);

        double yOut = yIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + 0.00001);
        yOut = (yOut + (yIn * 2)) / 3.0;
        return yOut;
    }

    public double getDriveX() {
        xIn = -getSpeedFromAxis(_leftJoystick, _leftJoystick.getXChannel());
        //        xIn = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_X.getNumber());
        xIn = Helpers.applyDeadband(xIn, DEADBAND);

        double xOut = xIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + 0.00001);
        xOut = (xOut + (xIn * 2)) / 3.0;
        return xOut;
    }

    public double getRotationX() {
        double speed = getSpeedFromAxis(_rightJoystick, _rightJoystick.getZChannel());
        speed = Helpers.applyDeadband(speed, 0.2);
        return speed;
    }

    protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

    //    public double getWinchSpeed() {
    //        double speed = getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_X.getNumber());
    //        speed = Helpers.applyDeadband(speed, 0.1);
    //        return speed;
    //    }
    //
    //    public double getHoodSpeed() {
    //        double speed = -getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_Y.getNumber());
    //        return speed;
    //    }

    public boolean raiseArm() {
        return _operatorAButton.get();
    }

    public boolean lowerArm() {
        return _operatorBButton.get();
    }

    @Override
    public void updateDashboard() {}
}
