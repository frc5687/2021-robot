/* (C)2021 */
package org.frc5687.infiniterecharge.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Intake extends OutliersSubsystem {

    private CANSparkMax _roller;
    private DoubleSolenoid _solenoid;

    public Intake(OutliersContainer container) {
        super(container);
        _roller =
                new CANSparkMax(
                        RobotMap.CAN.SPARKMAX.INTAKE, CANSparkMaxLowLevel.MotorType.kBrushless);
        _solenoid = new DoubleSolenoid(RobotMap.PCM.INTAKE_HIGH, RobotMap.PCM.INTAKE_LOW);

        _roller.restoreFactoryDefaults();
        _roller.setInverted(Constants.Intake.INVERTED);
        _roller.setIdleMode(CANSparkMax.IdleMode.kCoast);
        _roller.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        _roller.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 1000);
        _roller.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 1000);
        _solenoid.set(DoubleSolenoid.Value.kReverse); //Stops intake for droping
    }

    @Override
    public void periodic() {}

    public void setRollerSpeed(double pow) {
        //Sets intake roller speed
        //Invoked form AutoIntake
        _roller.set(pow);
    }

    public void intakeOff(){
        //Turn intake off
        _solenoid.set(DoubleSolenoid.Value.kOff);
    }

    public void raiseIntake() {
        //Raise intake
        _solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void lowerIntake() {
        //Lower intake
        _solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean isRaised() {
        //Is the intake up?
        //Returns true if solenoid is forward
        return _solenoid.get() == DoubleSolenoid.Value.kForward;
    }

    public boolean isLowered() {
        //Is the intake lowered
        //Returns true if solenoid is reversed
        return _solenoid.get() == DoubleSolenoid.Value.kReverse;
    }

    public Intake.Position getPosition() {
        //Get the in takes position
        DoubleSolenoid.Value current = _solenoid.get();
        if (current == Intake.Position.HIGH.getSolenoidValue()) {
            return Intake.Position.HIGH;
        } else if (current == Intake.Position.LOW.getSolenoidValue()) {
            return Intake.Position.LOW;
        }
        return Intake.Position.UNKNOWN;
    }

    @Override
    public void updateDashboard() {}

    private enum Position {
        UNKNOWN(DoubleSolenoid.Value.kOff),
        HIGH(DoubleSolenoid.Value.kReverse),
        LOW(DoubleSolenoid.Value.kForward);

        private DoubleSolenoid.Value solenoidValue;

        Position(DoubleSolenoid.Value solenoidValue) {
            this.solenoidValue = solenoidValue;
        }

        public DoubleSolenoid.Value getSolenoidValue() {
            return solenoidValue;
        }
    }
}
