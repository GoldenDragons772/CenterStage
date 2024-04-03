package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helper.TrajectoryManager;
import org.firstinspires.ftc.teamcode.opmode.Auto;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PropThresholdPipeline extends OpenCvPipeline {

    public enum propPos {
        LEFT,
        CENTER,
        RIGHT
    }

    Telemetry telemetry;

    private int spikeX = 0;

    private boolean debug = false;

    Mat mat = new Mat();
    Mat thresh = new Mat();

    Mat edges = new Mat();

    Mat hierarchy = new Mat();

    public PropThresholdPipeline(Telemetry telemetry, boolean debug) {
        this.debug = debug;
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        // "Mat" stands for matrix, which is basically the image that the detector will process
        // the input matrix is the image coming from the camera
        // the function will return a matrix to be drawn on your phone's screen

        // The detector detects regular stones. The camera fits two stones.
        // If it finds one regular stone then the other must be the skystone.
        // If both are regular stones, it returns NONE to tell the robot to keep looking

        // Make a working copy of the input matrix in HSV

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // We create a HSV range for yellow to detect regular stones
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value

        Scalar lowHSV; // lower bound HSV for Red
        Scalar highHSV; // higher bound HSV for Red

        if(Auto.alliance == TrajectoryManager.Alliance.RED) {
            lowHSV = new Scalar(0, 41, 96); // RED
            highHSV = new Scalar(5, 234, 202); // RED
        } else {
            lowHSV = new Scalar(111, 196, 70); // lower bound HSV for Blue\ new Scalar(111, 196, 70);
            highHSV = new Scalar(118, 255, 255); // higher bound HSV for Blue\ new Scalar(118, 255, 255);
        }


        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(mat, lowHSV, highHSV, thresh);

        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis

        // Blur the Image alot to reduce noise

        return thresh; // return the mat with rectangles drawn
    }


    public propPos getCurrentPropPos() {
        if ((Auto.alliance == TrajectoryManager.Alliance.RED && Auto.distance == TrajectoryManager.Distance.SHORT) || (Auto.alliance == TrajectoryManager.Alliance.BLUE && Auto.distance == TrajectoryManager.Distance.LONG)) {
            if(spikeX > 1 && spikeX < 150) {
                return propPos.LEFT;
            } else if(spikeX > 150 && spikeX < 500) {
                return propPos.CENTER;
            } else {
                return propPos.RIGHT;
            }
        } else {
            if(spikeX > 100 && spikeX < 300) {
                return propPos.LEFT;
            } else if(spikeX > 300 && spikeX < 600) {
                return propPos.CENTER;
            } else {
                return propPos.CENTER;
            }
        }
    }

    public String propPosString() {
        propPos currPos = getCurrentPropPos();
        if(currPos == propPos.LEFT) {
            return "PROP_LEFT";
        } else if(currPos == propPos.CENTER) {
            return "PROP_CENTER";
        } else if(currPos == propPos.RIGHT) {
            return  "PROP_RIGHT";
        }
        return "NOT_FOUND";
    };
}
