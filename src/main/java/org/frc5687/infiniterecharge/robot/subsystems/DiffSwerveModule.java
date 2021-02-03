/* (C)2020-2021 */
package org.frc5687.infiniterecharge.robot.subsystems;

import static org.frc5687.infiniterecharge.robot.Constants.DifferentialSwerveModule.TIMEOUT;
import static org.frc5687.infiniterecharge.robot.Constants.DifferentialSwerveModule.kDt;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.controller.ControllerUtil;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.*;
import edu.wpi.first.wpiutil.math.numbers.*;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.util.Helpers;

public class DiffSwerveModule {
    private final TalonFX _rightFalcon;
    private final TalonFX _leftFalcon;
    private final AnalogEncoder _lampreyEncoder;
    private final Translation2d _positionVector;
    private final LinearSystemLoop<N3, N2, N2> _swerveControlLoop;
    private Matrix<N3, N1> _reference; // same thing as a set point.
    private Matrix<N3, N1> _prevReference;
    private Matrix<N2, N1> _u;
    private double _vel;
    private double _positionError;

    public DiffSwerveModule(
            Translation2d positionVector,
            int leftMotorID,
            int rightMotorID,
            AnalogInput encoderNum) {
        _lampreyEncoder = new AnalogEncoder(encoderNum);
        _lampreyEncoder.setDistancePerRotation(
                2.0 * Math.PI / Constants.DifferentialSwerveModule.VOLTS_TO_ROTATIONS);
        _reference = Matrix.mat(Nat.N3(), Nat.N1()).fill(0, 0, 0);
        _prevReference = Matrix.mat(Nat.N3(), Nat.N1()).fill(0, 0, 0);
        _positionVector = positionVector;

        _leftFalcon = new TalonFX(leftMotorID);
        _rightFalcon = new TalonFX(rightMotorID);
        _rightFalcon.setInverted(false);
        _leftFalcon.setInverted(false);
        _rightFalcon.setSensorPhase(false);
        _leftFalcon.setSensorPhase(false);
        _rightFalcon.setNeutralMode(NeutralMode.Brake);
        _leftFalcon.setNeutralMode(NeutralMode.Brake);

        _rightFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, TIMEOUT);
        _leftFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, TIMEOUT);
        _rightFalcon.configForwardSoftLimitEnable(false);
        _leftFalcon.configForwardSoftLimitEnable(false);

        _leftFalcon.configVoltageCompSaturation(12.0, TIMEOUT);
        _rightFalcon.configVoltageCompSaturation(12.0, TIMEOUT);
        _leftFalcon.enableVoltageCompensation(true);
        _rightFalcon.enableVoltageCompensation(true);

        // Creates a Linear System of our Differential Swerve Module.
        LinearSystem<N3, N2, N2> swerveModuleModel =
                createDifferentialSwerveModule(
                        DCMotor.getFalcon500(2),
                        Constants.DifferentialSwerveModule.INERTIA_STEER,
                        Constants.DifferentialSwerveModule.INERTIA_WHEEL,
                        Constants.DifferentialSwerveModule.GEAR_RATIO_STEER,
                        Constants.DifferentialSwerveModule.GEAR_RATIO_WHEEL);

        // Creates a Kalman Filter as our Observer for our module. Works since system is linear.
        KalmanFilter<N3, N2, N2> swerveObserver =
                new KalmanFilter<>(
                        Nat.N3(),
                        Nat.N2(),
                        swerveModuleModel,
                        Matrix.mat(Nat.N3(), Nat.N1())
                                .fill(
                                        Units.degreesToRadians(
                                                Constants.DifferentialSwerveModule
                                                        .MODEL_AZIMUTH_ANGLE_NOISE),
                                        Units.rotationsPerMinuteToRadiansPerSecond(
                                                Constants.DifferentialSwerveModule
                                                        .MODEL_AZIMUTH_ANG_VELOCITY_NOISE),
                                        Units.rotationsPerMinuteToRadiansPerSecond(
                                                Constants.DifferentialSwerveModule
                                                        .MODEL_WHEEL_ANG_VELOCITY_NOISE)),
                        Matrix.mat(Nat.N2(), Nat.N1())
                                .fill(
                                        Units.degreesToRadians(
                                                Constants.DifferentialSwerveModule
                                                        .SENSOR_AZIMUTH_ANGLE_NOISE),
                                        Units.rotationsPerMinuteToRadiansPerSecond(
                                                Constants.DifferentialSwerveModule
                                                        .SENSOR_WHEEL_ANG_VELOCITY_NOISE)),
                        kDt);
        // Creates an LQR controller for our Swerve Module.
        LinearQuadraticRegulator<N3, N2, N2> swerveController =
                new LinearQuadraticRegulator<>(
                        swerveModuleModel,
                        // Q Vector/Matrix Maximum error tolerance
                        VecBuilder.fill(
                                Constants.DifferentialSwerveModule.Q_AZIMUTH,
                                Constants.DifferentialSwerveModule.Q_AZIMUTH_ANG_VELOCITY,
                                Constants.DifferentialSwerveModule.Q_WHEEL_ANG_VELOCITY),
                        // R Vector/Matrix Maximum control effort.
                        VecBuilder.fill(
                                Constants.DifferentialSwerveModule.CONTROL_EFFORT,
                                Constants.DifferentialSwerveModule.CONTROL_EFFORT),
                        kDt);

        // Creates a LinearSystemLoop that contains the Model, Controller, Observer, Max Volts,
        // Update Rate.
        _swerveControlLoop =
                new LinearSystemLoop<>(
                        swerveModuleModel, swerveController, swerveObserver, 12.0, kDt);

        _rightFalcon.setStatusFramePeriod(StatusFrame.Status_1_General, 5, TIMEOUT);
        _leftFalcon.setStatusFramePeriod(StatusFrame.Status_1_General, 5, TIMEOUT);
        _rightFalcon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, TIMEOUT);
        _leftFalcon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, TIMEOUT);
        _rightFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, TIMEOUT);
        _leftFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, TIMEOUT);
        _rightFalcon.configForwardSoftLimitEnable(false);
        _leftFalcon.configForwardSoftLimitEnable(false);
        _rightFalcon.configClosedloopRamp(0);
        _leftFalcon.configClosedloopRamp(0);
        _leftFalcon.configVoltageCompSaturation(12.0, TIMEOUT);
        _rightFalcon.configVoltageCompSaturation(12.0, TIMEOUT);
        _leftFalcon.enableVoltageCompensation(true);
        _rightFalcon.enableVoltageCompensation(true);
        _leftFalcon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms, TIMEOUT);
        _rightFalcon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms, TIMEOUT);
        _swerveControlLoop.reset(VecBuilder.fill(0, 0, 0));
        _u = VecBuilder.fill(0, 0);
        _positionError = 0;
        _vel = 0;
    }

    /**
     * wraps angle so that absolute encoder can be continues. (i.e) No issues when switching between
     * -PI and PI as they are the same point but different values.
     *
     * @param reference is the Matrix that contains the reference wanted such as [Math.PI, 0, 100].
     * @param xHat is the predicted states of our system. [Azimuth Angle, Azimuth Angular Velocity,
     *     Wheel Angular Velocity].
     * @param minAngle is the minimum angle in our case -PI.
     * @param maxAngle is the maximum angle in our case PI.
     */
    private Matrix<N3, N1> wrapAngle(
            Matrix<N3, N1> reference, Matrix<N3, N1> xHat, double minAngle, double maxAngle) {

        _positionError =
                ControllerUtil.getModulusError(
                        reference.get(0, 0), getModuleAngle(), minAngle, maxAngle);
        Matrix<N3, N1> error = reference.minus(xHat);
        double angVelError = reference.get(1, 0) - getAzimuthAngularVelocity();
        return VecBuilder.fill(_positionError, angVelError, error.get(2, 0));
    }

    public void periodic() {
        if (Math.abs(_vel - _reference.get(2, 0)) >= 3
                && _reference.get(2, 0) > _prevReference.get(2, 0)) {
            _vel += _reference.get(2, 0) * 0.01;
        } else if (Math.abs(_vel - _reference.get(2, 0)) >= 3
                && _reference.get(2, 0) < _prevReference.get(2, 0)) {
            _vel -= _reference.get(2, 0) * 0.01;
        } else {
            _vel = _reference.get(2, 0);
        }
        _swerveControlLoop.setNextR(_reference);

        _swerveControlLoop.correct(VecBuilder.fill(getModuleAngle(), getWheelAngularVelocity()));
        predict();
    }

    // use custom predict() function for as absolute encoder azimuth angle and the angular velocity
    // of the module need to be continuous.
    private void predict() {
        _u =
                _swerveControlLoop.clampInput(
                        _swerveControlLoop
                                .getController()
                                .getK()
                                .times(
                                        wrapAngle(
                                                _swerveControlLoop.getNextR(),
                                                _swerveControlLoop.getXHat(),
                                                -Math.PI,
                                                Math.PI)));
        _swerveControlLoop.getObserver().predict(_u, kDt);
    }

    public void setRightFalcon(double speed) {
        _rightFalcon.set(ControlMode.PercentOutput, speed);
    }

    public void setLeftFalcon(double speed) {
        _leftFalcon.set(ControlMode.PercentOutput, speed);
    }

    public void setRightFalconVoltage(double voltage) {
        double limVoltage = Helpers.limit(voltage, -12.0, 12.0);
        _rightFalcon.set(TalonFXControlMode.PercentOutput, limVoltage / 12.0);
    }

    public void setLeftFalconVoltage(double voltage) {
        double limVoltage = Helpers.limit(voltage, -12.0, 12.0);
        _leftFalcon.set(TalonFXControlMode.PercentOutput, limVoltage / 12.0);
    }

    public void setVelocityRPM(double RPM) {
        _rightFalcon.set(
                ControlMode.Velocity,
                (RPM
                        * Constants.DifferentialSwerveModule.TICKS_TO_ROTATIONS
                        / 600
                        / Constants.DifferentialSwerveModule.GEAR_RATIO_WHEEL));
        _leftFalcon.set(
                ControlMode.Velocity,
                (RPM
                        * Constants.DifferentialSwerveModule.TICKS_TO_ROTATIONS
                        / 600
                        / Constants.DifferentialSwerveModule.GEAR_RATIO_WHEEL));
    }

    public double getModuleAngle() {
        return Helpers.boundHalfAngle(_lampreyEncoder.getDistance(), true);
    }

    public double getLampreyVoltage() {
        return _lampreyEncoder.get();
    }

    public double getWheelAngularVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(
                        getLeftFalconRPM() / Constants.DifferentialSwerveModule.GEAR_RATIO_WHEEL
                                - getRightFalconRPM()
                                        / Constants.DifferentialSwerveModule.GEAR_RATIO_WHEEL)
                / 2.0;
    }

    public double getPositionError() {
        return _positionError;
    }

    public double getWheelVelocity() {
        return getWheelAngularVelocity()
                * Constants.DifferentialSwerveModule.WHEEL_RADIUS; // Meters per sec.
    }

    public double getAzimuthAngularVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(
                        getLeftFalconRPM() / Constants.DifferentialSwerveModule.GEAR_RATIO_STEER
                                + getRightFalconRPM()
                                        / Constants.DifferentialSwerveModule.GEAR_RATIO_STEER)
                / 2.0;
    }

    public double getRightFalconRPM() {
        return _rightFalcon.getSelectedSensorVelocity()
                / Constants.DifferentialSwerveModule.TICKS_TO_ROTATIONS
                * Constants.DifferentialSwerveModule.FALCON_RATE;
    }

    public double getLeftFalconRPM() {
        return _leftFalcon.getSelectedSensorVelocity()
                / Constants.DifferentialSwerveModule.TICKS_TO_ROTATIONS
                * Constants.DifferentialSwerveModule.FALCON_RATE;
    }

    public double getLeftVoltage() {
        return _leftFalcon.getMotorOutputVoltage();
    }

    public double getRightVoltage() {
        return _rightFalcon.getMotorOutputVoltage();
    }

    public double getPredictedAzimuthAngularVelocity() {
        return _swerveControlLoop.getObserver().getXhat(1);
    }

    public double getPredictedWheelAngularVelocity() {
        return _swerveControlLoop.getXHat(2);
    }

    public double getPredictedAzimuthAngle() {
        return _swerveControlLoop.getXHat(0);
    }

    public double getReferenceWheelAngularVelocity() {
        return _swerveControlLoop.getNextR(2);
    }

    public void setReference(Matrix<N3, N1> reference) {
        _prevReference = reference;
        _reference = reference;
    }

    /**
     * gets the wanted voltage from our control law. u = K(r-x) our control law is slightly
     * different as we need to be continuous. Check method predict() for calculations.
     *
     * @return left wanted voltage
     */
    public double getLeftNextVoltage() {
        return _u.get(0, 0);
    }

    public double getRightNextVoltage() {
        return _u.get(1, 0);
    }

    public double getReferenceModuleAngle() {
        return _swerveControlLoop.getNextR(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getWheelVelocity(), new Rotation2d(getModuleAngle()));
    }

    public double[] getReference() {
        return _reference.getData();
    }

    /**
     * Sets the state of the module and sends the voltages wanted to the motors.
     *
     * @param state is the desired swerve module state.
     */
    public void setModuleState(SwerveModuleState state) {
        setReference(
                VecBuilder.fill(
                        state.angle.getRadians(),
                        0,
                        state.speedMetersPerSecond
                                / Constants.DifferentialSwerveModule.WHEEL_RADIUS));
        setLeftFalconVoltage(getLeftNextVoltage());
        setRightFalconVoltage(getRightNextVoltage());
    }

    public void setIdealState(SwerveModuleState state) {
        Rotation2d angleDifference = state.angle.minus(new Rotation2d(getModuleAngle()));
        if (Math.abs(angleDifference.getRadians()) > Math.PI / 2.0) {
            setModuleState(
                    new SwerveModuleState(
                            -state.speedMetersPerSecond,
                            state.angle.rotateBy(new Rotation2d(Math.PI))));
        } else {
            setModuleState(state);
        }
    }

    /**
     * Creates a StateSpace model of a differential swerve module.
     *
     * @param motor is the motor used.
     * @param Js is the Moment of Inertia of the steer component.
     * @param Jw is the Moment of Inertia of the wheel component.
     * @param Gs is the Gear Ratio of the steer.
     * @param Gw is the Gear Ratio of the wheel.
     * @return LinearSystem of state space model.
     */
    private static LinearSystem<N3, N2, N2> createDifferentialSwerveModule(
            DCMotor motor, double Js, double Jw, double Gs, double Gw) {
        var Cs = -((Gs * motor.KtNMPerAmp) / (motor.KvRadPerSecPerVolt * motor.rOhms * Js));
        var Cw = -((Gw * motor.KtNMPerAmp) / (motor.KvRadPerSecPerVolt * motor.rOhms * Jw));
        var Vs = 0.5 * ((Gs * motor.KtNMPerAmp) / (motor.rOhms * Js));
        var Vw = 0.5 * ((Gw * motor.KtNMPerAmp) / (motor.rOhms * Jw));

        var A =
                Matrix.mat(Nat.N3(), Nat.N3())
                        .fill(0.0, 1.0, 0.0, 0.0, Gs * Cs, 0.0, 0.0, 0.0, Gw * Cw);
        var B = Matrix.mat(Nat.N3(), Nat.N2()).fill(0.0, 0.0, Vs, Vs, Vw, -Vw);
        var C = Matrix.mat(Nat.N2(), Nat.N3()).fill(1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        var D =
                Matrix.mat(Nat.N2(), Nat.N2())
                        .fill(
                                0.0, 0.0,
                                0.0, 0.0);
        return new LinearSystem<>(A, B, C, D);
    }
}
