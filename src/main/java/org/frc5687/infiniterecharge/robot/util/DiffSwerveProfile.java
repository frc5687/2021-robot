/* (C)2021 */
package org.frc5687.infiniterecharge.robot.util;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.*;
import org.frc5687.infiniterecharge.robot.Constants;

// Utility class to hold all of the motion profiles for [azimuth, azimuth ang_vel, wheel ang_vel]
public class DiffSwerveProfile {

    private TrapezoidProfile.Constraints _constraint;
    private TrapezoidProfile.State _azimuthGoal;
    private TrapezoidProfile.State _azimuthSetpoint;

    private Matrix<N3, N1> _reference;

    public DiffSwerveProfile() {
        _constraint =
                new TrapezoidProfile.Constraints(
                        Constants.DifferentialSwerveModule.TRAP_ANG_VELOCITY,
                        Constants.DifferentialSwerveModule.TRAP_ANG_ACCEL);
        _azimuthGoal = new TrapezoidProfile.State();
        _azimuthSetpoint = new TrapezoidProfile.State();
        _reference = VecBuilder.fill(0, 0, 0);
    }

    public void calculate(Matrix<N3, N1> reference, double kDt) {
        _azimuthGoal = new TrapezoidProfile.State(reference.get(0, 0), 0);
        TrapezoidProfile profile =
                new TrapezoidProfile(_constraint, _azimuthGoal, _azimuthSetpoint);
        _azimuthSetpoint = profile.calculate(kDt);
        _reference =
                VecBuilder.fill(
                        _azimuthSetpoint.position, _azimuthSetpoint.velocity, _reference.get(2, 0));
    }

    public Matrix<N3, N1> reference() {
        return _reference;
    }
}
