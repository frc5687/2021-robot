package org.frc5687.infiniterecharge.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Climber extends OutliersSubsystem {
    private CANSparkMax _winch;
    private CANEncoder _winchEncoder;

    public Climber(OutliersContainer container ) {
        super(container);
        _winch = new CANSparkMax(RobotMap.CAN.SPARKMAX.WINCH, CANSparkMaxLowLevel.MotorType.kBrushless);
        _winchEncoder = _winch.getEncoder();
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void updateDashboard() {
    }

    public void setWinchSpeed(double speed) {
        _winch.set(speed);
    }
    public double getWinchPosition() {
        return _winchEncoder.getPosition();
    }

    public double getWinchRPM() {
        return _winchEncoder.getVelocity();
    }

}
