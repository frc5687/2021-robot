/* (C)5687-2021 */
package org.frc5687.infiniterecharge.robot;

import static org.frc5687.infiniterecharge.robot.Constants.DriveTrain.*;
import static org.frc5687.infiniterecharge.robot.Constants.EPSILON;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.frc5687.infiniterecharge.robot.commands.*;
import org.frc5687.infiniterecharge.robot.commands.climber.Climb;
import org.frc5687.infiniterecharge.robot.commands.climber.LowerArm;
import org.frc5687.infiniterecharge.robot.commands.climber.RaiseArm;
import org.frc5687.infiniterecharge.robot.commands.climber.ResetWinch;
import org.frc5687.infiniterecharge.robot.commands.shooter.Shoot;
import org.frc5687.infiniterecharge.robot.subsystems.*;
import org.frc5687.infiniterecharge.robot.util.*;
import org.frc5687.infiniterecharge.robot.util.Limelight;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected Gamepad _operatorGamepad;
    protected Joystick _leftJoystick;
    protected Joystick _rightJoystick;
    protected Joystick _raceWheel;

    protected Button _driverRightStickButton;

    private final JoystickButton _aimButton;
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
    private final Button _operatorRightYUp;
    private final Button _operatorRightYDown;
    private final Button _operatorLeftYUp;
    private final Button _operatorLeftYDown;
    private double yIn = 0;
    private double xIn = 0;

    public OI() {
        _driverGamepad = new Gamepad(0);
        _operatorGamepad = new Gamepad(3);

        _leftJoystick = new Joystick(1);
        _rightJoystick = new Joystick(2);

        _raceWheel = new Joystick(4);

        _driverRightStickButton =
                new JoystickButton(_driverGamepad, Gamepad.Buttons.RIGHT_STICK.getNumber());

        _aimButton = new JoystickButton(_rightJoystick, 1);
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

        _operatorRightYUp =
                new AxisButton(_operatorGamepad, Gamepad.Axes.RIGHT_Y.getNumber(), -0.4);

        _operatorRightYDown =
                new AxisButton(_operatorGamepad, Gamepad.Axes.RIGHT_Y.getNumber(), 0.4);

        _operatorLeftYUp = new AxisButton(_operatorGamepad, Gamepad.Axes.LEFT_Y.getNumber(), -0.4);
        _operatorLeftYDown = new AxisButton(_operatorGamepad, Gamepad.Axes.LEFT_Y.getNumber(), 0.4);
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
            Climber climber,
            Trajectory prt1,
            Trajectory prt2,
            Trajectory prt3,
            Trajectory prt4) {
        _operatorRightTrigger.whileHeld(new Shoot(shooter, spindexer, hood));
        _operatorLeftTrigger.whileHeld(new AutoIntake(intake));
        _aimButton.whileHeld(new AutoTarget(drivetrain, shooter, hood, this, 65, 5000, true));
        // Climber Stuff:
        //X Box controller
        _operatorAButton.whenPressed(new RaiseArm(climber, shooter));
        _operatorBButton.whenPressed(new LowerArm(climber));
        _operatorYButton.whenPressed(new ResetWinch(climber));
        _operatorXButton.whenPressed(new Climb(climber));
        //DPad
        _operatorRightYUp.whenPressed(new RaiseArm(climber, shooter));
        _operatorRightYDown.whenPressed(new LowerArm(climber));
        _operatorLeftYUp.whileHeld(new ResetWinch(climber));
        _operatorLeftYDown.whileHeld(new Climb(climber));
    }

    public double getDriveY() {
        yIn = getSpeedFromAxis(_leftJoystick, _leftJoystick.getYChannel());
        //        yIn = getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        yIn = Helpers.applyDeadband(yIn, DEADBAND);

        double yOut = yIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + EPSILON);
        yOut = (yOut + (yIn * 2)) / 3.0;
        return yOut;
    }

    public double getDriveX() {
        xIn = -getSpeedFromAxis(_leftJoystick, _leftJoystick.getXChannel());
        //        xIn = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_X.getNumber());
        xIn = Helpers.applyDeadband(xIn, DEADBAND);

        double xOut = xIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + EPSILON);
        xOut = (xOut + (xIn * 2)) / 3.0;
        return xOut;
    }

    public double getRotationX() {
        double speed = -getSpeedFromAxis(_raceWheel, _raceWheel.getZChannel());
        speed = Helpers.applyDeadband(speed, 0.2);
        return speed;
    }

    protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

    public int getOperatorPOV() {
        return POV.fromWPILIbAngle(0, _operatorGamepad.getPOV()).getDirectionValue();
    }

    public boolean isKillAllPressed() {
        int operatorPOV = getOperatorPOV();
        return operatorPOV == Constants.OI.KILL_ALL;
    }

    @Override
    public void updateDashboard() {}
}
