/* (C)2021 */
package org.frc5687.infiniterecharge.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import java.io.*;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
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
    private boolean closeProxy_ = false;
    private Pose2d initPose = new Pose2d();

    private JetsonListener _jetsonListener;
    private Timer _jetsonTimer;

    public JetsonProxy(int period) {
        _period = period;
        try {
            server = new ServerSocket(_rioPort);
            DriverStation.reportError("Socket Created", false);
            //            socket = server.accept();
        } catch (IOException exception) {
            DriverStation.reportError("Error: " + exception.getMessage(), true);
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
        if (_latestFrame == null) {
            return new Frame("");
        }
        return _latestFrame;
    }

    public class Frame {
        private long _millis;
        private double _estimatedX;
        private double _estimatedY;
        private double _estimatedHeading;

        public Frame(String packet) {
            if (packet.equals("")) {
                _millis = 0;
                _estimatedX = 0;
                _estimatedY = 0;
                _estimatedHeading = 0;
            } else {
                String[] a = packet.split(";");
                _millis = Long.parseLong(a[0]);
                _estimatedX = Double.parseDouble(a[2]);
                _estimatedY = Double.parseDouble(a[3]);
                _estimatedHeading = Double.parseDouble(a[4]);
            }
        }

        public long getMillis() {
            return _millis;
        }

        public Pose2d getEstimatedPose() {
            return new Pose2d(_estimatedX, _estimatedY, Rotation2d.fromDegrees(_estimatedHeading));
        }
    }

    public static class OutFrame {
        String data = "";

        public OutFrame(Pose2d initPose, SwerveModuleState[] moduleStates) {
            StringBuilder buffer = new StringBuilder();
            buffer.append(System.currentTimeMillis());
            buffer.append(";");
            buffer.append(initPose.getX());
            buffer.append(";");
            buffer.append(initPose.getY());
            buffer.append(";");
            buffer.append(initPose.getRotation().getRadians());
            for (SwerveModuleState state : moduleStates) {
                buffer.append(";");
                buffer.append(state.speedMetersPerSecond);
                buffer.append(";");
                buffer.append(state.angle.getRadians());
            }
            buffer.append(";");
            data = buffer.toString();
        }

        public String getString() {
            return data;
        }
    }

    public void sendOutFrame(OutFrame frame) {
        try {
            OutputStream output = socket.getOutputStream();
            PrintWriter writer = new PrintWriter(output, true);
            writer.println(frame.getString());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void setInitPose(Pose2d pose) {
        initPose = pose;
    }

    public Pose2d getInitPose() {
        return initPose;
    }

    public boolean isSocketNull() {
        return socket == null;
    }

    protected class JetsonListener implements Runnable {
        private JetsonProxy _proxy;
        private InetAddress _jetsonAddress = null;
        private InputStream _incomingFrame;
        private int _roboRioPort;

        protected JetsonListener(JetsonProxy proxy, int roboRioPort) {
            _roboRioPort = roboRioPort;
            _proxy = proxy;
        }

        @Override
        public void run() {
            try {
                socket = server.accept();
                byte[] data = new byte[1024];

                while (true) {
                    _incomingFrame = socket.getInputStream();
                    _incomingFrame.read(data);
                    String raw = new String(data, StandardCharsets.UTF_8);
                    Frame frame = new Frame(raw);
                    _proxy.setLatestFrame(frame);
                    //                    _incomingFrame.reset();
                }
            } catch (IOException e) {
                DriverStation.reportError("died 1" + e.getMessage(), true);
            } catch (Exception e) {
                DriverStation.reportError("died 2" + e.getMessage(), true);
            }
        }
    }
}
