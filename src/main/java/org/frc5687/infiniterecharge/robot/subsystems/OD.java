package org.frc5687.infiniterecharge.robot.subsystems;

import org.frc5687.infiniterecharge.robot.util.MetricTracker;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Scalar;  
import org.opencv.features2d.FastFeatureDetector;
import org.opencv.features2d.Feature2D;
import org.opencv.features2d.Features2d;
import org.opencv.videoio.VideoCapture;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import org.opencv.highgui.HighGui;

public class OD {
    public static MetricTracker _metric;
    public static final int JETSON_PORT = 27002;
    public static final int RIO_PORT = 27001;
    public static final int PERIOD = 10;
    public static int capCount = 1;
    public static double FPS = 0;

    public static void USBCam(){
        // Creates UsbCamera and MjpegServer [1] and connects them
        UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
        MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);
        mjpegServer1.setSource(usbCamera);

        // Creates the CvSink and connects it to the UsbCamera
        CvSink cvSink = new CvSink("opencv_USB Camera 0");
        cvSink.setSource(usbCamera);

        // Creates the CvSource and MjpegServer [2] and connects them
        CvSource outputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);
        MjpegServer mjpegServer2 = new MjpegServer("serve_Blur", 1182);
        mjpegServer2.setSource(outputStream);
        FPS = outputStream.getActualFPS();
    }

    public static void Capture(){
        //Setup video capture
        VideoCapture cap = new VideoCapture(0);
        //Create new frame
        Mat frame = new Mat();
        int index = 0;
        //So long as the cam is running let's take some pictures
        if(cap.isOpened()){
            _metric.put("CVCam", 1);
            while(true){
                cap.read(frame);
                capCount = capCount + 1;
                _metric.put("Caps", capCount);
                HighGui.imshow("Frame", frame);
                index = HighGui.waitKey(1);
                if(index == 27){
                    break;
                }
            }
        }else{
            //Error has happened and cam is no go
            _metric.put("CVCam", 0);
        }
    }

    public static void GetImportantPoints(Mat frame){
        //New image
        Mat output = new Mat();
        //Make map of key ponits image
        MatOfKeyPoint matOfKeyPoints = new MatOfKeyPoint();
        //Create feature points map
        FastFeatureDetector featureDetector = FastFeatureDetector.create();
        featureDetector.detect(frame, matOfKeyPoints);
        Features2d.drawKeypoints(frame, matOfKeyPoints, output, new Scalar(0, 0, 255));
        //Show feature map
        HighGui.imshow("Feature Detection", output);
        HighGui.waitKey();
    }

    /*public static void ObjectDetect(){
        System.loadLibrary("opencv_java340");
        DeepNerualNetworkProcessor processor
    }*/
}
//Kilroy Was Here

