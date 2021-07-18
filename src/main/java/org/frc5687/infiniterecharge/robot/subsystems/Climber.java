/* (C)2021 */
package org.frc5687.infiniterecharge.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Climber extends OutliersSubsystem {
    private CANSparkMax _winch;
    private CANEncoder _winchEncoder;

    private DoubleSolenoid _arm;

    public Climber(OutliersContainer container) {
        super(container);
        _winch =
                new CANSparkMax(
                        RobotMap.CAN.SPARKMAX.WINCH, CANSparkMaxLowLevel.MotorType.kBrushless);
        _winchEncoder = _winch.getEncoder();
        _arm = new DoubleSolenoid(RobotMap.PCM.ARM_HIGH, RobotMap.PCM.ARM_LOW);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void updateDashboard() {}

    public void setWinchSpeed(double speed) {
        _winch.set(speed);
    }

    public void raiseArm() {
        _arm.set(DoubleSolenoid.Value.kForward);
    }

    public void lowerArm() {
        _arm.set(DoubleSolenoid.Value.kReverse);
    }

    public double getWinchPosition() {
        return _winchEncoder.getPosition();
    }

    public double getWinchRPM() {
        return _winchEncoder.getVelocity();
    }

    public Climber.Position getArmPosition() {
        DoubleSolenoid.Value current = _arm.get();
        if (current == Climber.Position.RAISED.getArmValue()) {
            return Position.RAISED;
        } else if (current == Position.LOWERED.getArmValue()) {
            return Climber.Position.LOWERED;
        }
        return Climber.Position.UNKNOWN;
    }

    private enum Position {
        UNKNOWN(DoubleSolenoid.Value.kOff),
        RAISED(DoubleSolenoid.Value.kForward),
        LOWERED(DoubleSolenoid.Value.kReverse);

        private DoubleSolenoid.Value armValue;

        Position(DoubleSolenoid.Value armValue) {
            this.armValue = armValue;
        }

        public DoubleSolenoid.Value getArmValue() {
            return armValue;
        }
    }
}
