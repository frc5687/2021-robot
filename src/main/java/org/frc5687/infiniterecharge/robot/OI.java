/* (C)5687-2021 */
package org.frc5687.infiniterecharge.robot;

import static org.frc5687.infiniterecharge.robot.Constants.DriveTrain.*;

import javax.sound.sampled.LineEvent;

import static org.frc5687.infiniterecharge.robot.Constants.EPSILON;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.frc5687.infiniterecharge.robot.commands.*;
import org.frc5687.infiniterecharge.robot.commands.climber.Climb;
import org.frc5687.infiniterecharge.robot.commands.climber.LowerArm;
import org.frc5687.infiniterecharge.robot.commands.climber.RaiseArm;
import org.frc5687.infiniterecharge.robot.commands.climber.ResetWinch;
import org.frc5687.infiniterecharge.robot.commands.shooter.Shoot;
import org.frc5687.infiniterecharge.robot.subsystems.*;
import org.frc5687.infiniterecharge.robot.util.*;

public class OI extends OutliersProxy {
    
    //This is the fun raceing wheel thing
    private Joystick raceWheel;
    //Translation Joystick
    private Joystick translation;
    //Operator Xbox controller
    private XboxController operator;
    //For aiming the robot
    private JoystickButton aimBTN;

    private JoystickButton manualAim;
    //Shoot
    private JoystickButton shootBTN;

    private JoystickButton shimmyBTM;
    //XBox buttons
    //Drop intake
    private JoystickButton intakeBTN;
    //Raise Arm
    private JoystickButton climbUPBTN;
    //Lower Arm
    private JoystickButton climbDOWNBTN;
    //Reset winch
    private JoystickButton winchReset;
    //Climb
    private JoystickButton climb;
    //Maverick
    private JoystickButton Maverick;
    //Reset navX
    private JoystickButton resetNavX;
    //Joystick X values
    private double xIn;
    //Joystick Y values
    private double yIn;
    //Limelight
    private Limelight limeLight;

    public OI() {
            /**
            *The number in the () is the port maping
            *We need to figure out if each time the USBs...
            *are plugged in the port numbers change
            */
            raceWheel = new Joystick(1); //This hap-hazard port aloction is going to get someone killed 
            translation = new Joystick(2);
            operator = new XboxController(4); //Was 3
            /**
             * In the ()
             * 1: The object that's being maped to
             * 2: The index of the button being mapped
             */
            aimBTN = new JoystickButton(raceWheel, 5);
            manualAim = new JoystickButton(raceWheel, 1);
            shootBTN = new JoystickButton(translation, 1);
            shimmyBTM = new JoystickButton(raceWheel, 10);
            //Xbox buttons even though it's using the joystick class :)
            intakeBTN = new JoystickButton(raceWheel, 6); //A
            climbDOWNBTN = new JoystickButton(translation, 8); //Left Trigger
            climbUPBTN = new JoystickButton(raceWheel, 3); //Right Trigger  
            winchReset = new JoystickButton(operator, 8); //
            climb = new JoystickButton(translation, 9); //X/
            Maverick = new JoystickButton(translation, 4);
            resetNavX = new JoystickButton(translation, 5);
            //POV down
    }

    public void initializeButtons(DriveTrain drivetrain,
            Shooter shooter,
            Intake intake,
            Spindexer spindexer,
            Hood hood,
            Climber climber,  
            Trajectory prt1,
            Trajectory prt2,
            Trajectory prt3,
            Trajectory prt4) {
                    //#region BUTTON_BINDS
                    /*Binds buttons */
                    shootBTN.whenHeld(new Shoot(shooter, spindexer, hood));

                    // Auto target takes in 3 subsystems: drivetrain, shooter, hood, as well as OI.
                    // the two value arguments is the hood angle reference and flywheel rpm reference. These are only set
                    // if the last argument "override" is set to true. all this information can be inferred by the AutoTarget command class.
                    aimBTN.whenHeld(new AutoTarget(drivetrain, shooter, hood, this, 65, 5000, true));
                    manualAim.whenHeld(new ManualTarget(drivetrain, shooter, hood, this, 65, 5000, true));

                    /*Climber stuff*/
                    climbUPBTN.whenPressed(new RaiseArm(climber, shooter));
                    climbDOWNBTN.whenPressed(new LowerArm(climber));
                    winchReset.whenPressed(new ResetWinch(climber));
                    climb.whenHeld(new Climb(climber));
                    /*Intake */
                    intakeBTN.whenHeld(new AutoIntake(intake));
                    Maverick.whenHeld(new Maverick(drivetrain));
                    //#endregion
                    // Use lambda expression to call resetYaw function from drivetrain instead of calling an entire class.
                    resetNavX.whenPressed(drivetrain::resetYaw);

                    shimmyBTM.whenHeld(new ClearSpindexer(spindexer));
            }

            public double getDriveY() {
                yIn = getSpeedFromAxis(translation, translation.getYChannel());
                //        yIn = getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_Y.getNumber());
                yIn = Helpers.applyDeadband(yIn, DEADBAND);
        
                double yOut = yIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + EPSILON);
                yOut = (yOut + (yIn * 2)) / 3.0;
                return yOut;
            }
        
            public double getDriveX() {
                xIn = -getSpeedFromAxis(translation, translation.getXChannel());
                xIn = Helpers.applyDeadband(xIn, DEADBAND);
        
                double xOut = xIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + EPSILON);
                xOut = (xOut + (xIn * 2)) / 3.0;
                return xOut;
            }

            public double getRotationX() {
                double speed = -getSpeedFromAxis(raceWheel, raceWheel.getXChannel());
                speed = speed * Constants.DriveTrain.WHEEL_SPEED; //To make turning faster.
                speed = Helpers.applyDeadband(speed, RACE_WHEEL_DEADBAND); // The racewheel already has a built-in deadband.
                return speed;
            }
        
            protected double getSpeedFromAxis(Joystick _raceWheel, int axisNumber) {
                return _raceWheel.getRawAxis(axisNumber);
            }
        
            public int getOperatorPOV() {
                return POV.fromWPILIbAngle(0, operator.getPOV()).getDirectionValue();
            }
        
            public boolean isKillAllPressed() {
                int operatorPOV = getOperatorPOV();
                return operatorPOV == Constants.OI.KILL_ALL;
            }
        
            @Override
            public void updateDashboard() {}
        }
        

            /**
             * Here is the button list mapping for XBox controller:

Stick 1 = left analog stick
Stick 2 = right analog stick
POV = dpad
Button 1 = A
Button 2 = B
Button 3 = X
Button 4 = Y
Button 5 = LB
Button 6 = RB
Button 7 = back
Button 8 = start
Button 9 = left analog stick center pushed in
Button 10 = right analog stick center pushed in
Button 11 = left trigger
Button 12 = right trigger
Button 13 = X silver guide button
             */
