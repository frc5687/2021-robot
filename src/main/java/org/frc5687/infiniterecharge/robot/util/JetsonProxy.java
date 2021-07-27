/* (C)2021 */
package org.frc5687.infiniterecharge.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import java.io.*;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Timer;

public class JetsonProxy {

    public static final int JETSON_PORT = 27002;
    public static final int RIO_PORT = 27001;
    public static final int PERIOD = 10;

    private Socket socket;
    private ServerSocket server;
    private DataInputStream _input;
    private DataOutputStream _output;
    private Thread listenerThread;

    private int _period = PERIOD;
    private int _jetsonPort = JETSON_PORT;
    private int _rioPort = RIO_PORT;

    private Frame _latestFrame;

    private long trackingMillis = System.currentTimeMillis();

    private JetsonListener _jetsonListener;
    private Timer _jetsonTimer;

    public JetsonProxy(int period) {
        _period = period;
        try {
            server = new ServerSocket(_rioPort);
            socket = server.accept();
        } catch (IOException exception) {
            socket = null;
        }

        _jetsonListener = new JetsonListener(this, _rioPort);
        listenerThread = new Thread(_jetsonListener);
        listenerThread.start();
    }

    protected synchronized void setLatestFrame(Frame frame) {
        _latestFrame = frame;
    }

    public Frame getLatestFrame() {
        return _latestFrame;
    }

    public class Frame {
        private long _millis;
        private double _estimatedX;
        private double _estimatedY;
        private double _estimatedHeading;

        public Frame(String packet) {
            String[] a = packet.split(";");
            _millis = Long.parseLong(a[0]);
            _estimatedX = Double.parseDouble(a[2]);
            _estimatedY = Double.parseDouble(a[3]);
            _estimatedHeading = Double.parseDouble(a[4]);
        }

        public long getMillis() {
            return _millis;
        }

        public Pose2d getEstimatedPose() {
            return new Pose2d(_estimatedX, _estimatedY, Rotation2d.fromDegrees(_estimatedHeading));
        }
    }

    protected class JetsonListener implements Runnable {
        private JetsonProxy _proxy;
        private InetAddress _jetsonAddress = null;
        private BufferedReader _incomingFrame;
        private int _roboRioPort;

        protected JetsonListener(JetsonProxy proxy, int roboRioPort) {
            _roboRioPort = roboRioPort;
            _proxy = proxy;
        }

        @Override
        public void run() {
            String raw = null;
            try {
                _incomingFrame = new BufferedReader(new InputStreamReader(socket.getInputStream()));
                while (true) {
                    raw = _incomingFrame.readLine();
                    Frame frame = new Frame(raw);
                    //                    DriverStation.reportError("got message", false);
                    _proxy.setLatestFrame(frame);
                }
            } catch (IOException e) {
                DriverStation.reportError("died 1" + e.getMessage(), true);
            } catch (Exception e) {
                DriverStation.reportError("died 2" + e.getMessage(), true);
            }
        }
    }
}
