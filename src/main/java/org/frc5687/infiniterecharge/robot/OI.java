/* (C)5687-2021 */
package org.frc5687.infiniterecharge.robot;

import static org.frc5687.infiniterecharge.robot.Constants.DriveTrain.*;
import static org.frc5687.infiniterecharge.robot.Constants.EPSILON;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
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
        private JoystickButton aimBTN; //Auto aim robot
        private JoystickButton shootBTN; //Shoot balls
        //XBox button
        private JoystickButton intakeBTN; //Drop intake
        private JoystickButton climbUPBTN; //Raise Arm
        private JoystickButton climbDOWNBTN; //Lower Arm
        private JoystickButton winchReset; //Reset winch
        private JoystickButton climb; //Climb

        private double xIn; //Joystick X values
        private double yIn; //Joystick Y values
        

    public OI() {
            /**
            *The number in the () is the port maping
            *We need to figure out if each time the USBs...
            *are plugged in the port numbers change
            */
            raceWheel = new Joystick(0);
            translation = new Joystick(2);
            operator = new XboxController(4); //Was 3
            /**
             * In the ()
             * 1: The object that's being maped to
             * 2: The index of the button being mapped
             */
            aimBTN = new JoystickButton(raceWheel, 1);
            shootBTN = new JoystickButton(raceWheel, 2);
            //Xbox buttons even though it's using the joystick class :)
            intakeBTN = new JoystickButton(operator, 1); //A
            climbUPBTN = new JoystickButton(operator, 12); //Left Trigger
            climbDOWNBTN = new JoystickButton(operator, 11); //Right Trigger  
            winchReset = new JoystickButton(operator, 2); //B
            climb = new JoystickButton(operator, 3); //X
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

                    //We need to figure out what the heck these values after "hood" do
                    aimBTN.whenHeld(new AutoTarget(drivetrain, shooter, hood, this, 65, 5000, true)); 

                    /*Climber stuff*/
                    climbUPBTN.whenPressed(new RaiseArm(climber, shooter));
                    climbDOWNBTN.whenPressed(new LowerArm(climber));
                    winchReset.whenHeld(new ResetWinch(climber));
                    climb.whenPressed(new Climb(climber));
                    //Intake
                    intakeBTN.whenPressed(new AutoIntake(intake));
                    //#endregion
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
                speed = Helpers.applyDeadband(speed, 0); // The racewheel already has a built-in deadband.
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