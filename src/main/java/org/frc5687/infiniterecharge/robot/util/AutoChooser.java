/* (C)2021 */
package org.frc5687.infiniterecharge.robot.util;

import org.frc5687.infiniterecharge.robot.RobotMap;

/** Created by Ben Bernard on 2/2/2018. */
public class AutoChooser extends OutliersProxy {
    private RotarySwitch _modeSwitch;
    private final double TOLERANCE = 0.2;
    private static MetricTracker _metric;

    public AutoChooser(OutliersContainer.IdentityMode identityMode) {
        _modeSwitch = new RotarySwitch(RobotMap.Analog.MODE_SWITCH, TOLERANCE, 0.077, 0.154, 0.231, 0.308, 0.385, 0.462,
                0.538, 0.615, 0.693, 0.770, 0.847, 0.925);
        // _modeSwitch = new RotarySwitch(RobotMap.Analog.MODE_SWITCH,
        // Constants.RotarySwitch.TOLERANCE, .09, .17, .23, .31, .5, .59, .68, .75, .82,
        // .91, .96);
    }

    public Mode getSelectedMode() {
        int raw = _modeSwitch.get();
        if (raw >= Mode.values().length) {
            raw = 0;
        }
        try {
            return Mode.values()[raw];
        } catch (Exception e) {
            return Mode.StayPut;
        }
    }

    public void updateDashboard() {
        metric("Label/Mode", getSelectedMode().getLabel());
        metric("Raw/Mode", _modeSwitch.getRaw());
        metric("Numeric/Mode", _modeSwitch.get());
    }

    public enum Mode {
        StayPut(0, "Stay Put"), ShootAndGo(1, "Shoot and Cross"), ShootAndNearTrench(2, "Shoot and Near Trench"),
        ShootAndFarTrench(3, "Shoot and Far Trench"), Generator2NearTrench(4, "Generator 2 and Near Trench"),
        Generator2FarTrench(5, "Generator 2 and Far Trench"), SnipeAndNearTrench(6, "Snipe and Near Trench"),
        StealTenBall(7, "Steal and Generator Balls");

        private String _label;
        private int _value;

        Mode(int value, String label) {
            _value = value;
            _label = label;
            _metric.put("Auto Mode", label);
        }

        public int getValue() {
            return _value;
        }

        public String getLabel() {
            return _label;
        }
    }
}
