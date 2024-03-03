package org.firstinspires.ftc.teamcode.vision.sim;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.helper.TrajectoryManager;
//import org.firstinspires.ftc.teamcode.opmode.Auto;
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

public class PropSim extends OpenCvPipeline {

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

    public PropSim(Telemetry telemetry) {
        this.debug = true;
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

//        if(Auto.alliance == TrajectoryManager.Alliance.RED) {
            lowHSV = new Scalar(0, 41, 96); // RED
            highHSV = new Scalar(5, 234, 202); // RED
//        } else {
//            lowHSV = new Scalar(111, 196, 70); // lower bound HSV for Blue
//            highHSV = new Scalar(118, 255, 255); // higher bound HSV for Blue
//        }


        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(mat, lowHSV, highHSV, thresh);

        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis

        // Blur the Image alot to reduce noise
        Imgproc.GaussianBlur(thresh, thresh, new org.opencv.core.Size(5, 5), 1);

        Imgproc.Canny(thresh, edges, 200, 255);

        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // Oftentimes the edges are disconnected. findContours connects these edges.
        // We then find the bounding rectangles of those contours
        // Merge the rectangles to form a single rectangle
        List<MatOfPoint> contours = new ArrayList<>();

        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        // Iterate and check whether the bounding boxes
        // cover left and/or right side of the image

        // Draw one contour for each rectangle

        // SetDefault
        spikeX = 0;

        for (int i = 0; i != boundRect.length; i++) {
            // If the area of the rectangle is greater than 2000, then its a Spike
            if (boundRect[i].area() > 3000) {

                int y = boundRect[i].y + boundRect[i].height / 2;

                if(y > 150) {
                    Imgproc.rectangle(mat, boundRect[i], new Scalar(255, 255, 255), 2);

                    spikeX = boundRect[i].x + boundRect[i].width / 2;

                    // Draw a circle at the center of the rectangle
                    Imgproc.circle(mat, new Point(spikeX, y), 5, new Scalar(255, 255, 255), 1);
                }
            }
        }

        if(debug) {
            // Print the center of the rectangle
            telemetry.addData("Prop Position", spikeX);
            //telemetry.addData("Prop Y Position", y);
//            telemetry.addData("Prop Location", propPosString());
            telemetry.update();
        }
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_HSV2RGB);

        return mat; // return the mat with rectangles drawn
    }


//    public propPos getCurrentPropPos() {
//        if ((Auto.alliance == TrajectoryManager.Alliance.RED && Auto.distance == TrajectoryManager.Distance.SHORT) || (Auto.alliance == TrajectoryManager.Alliance.BLUE && Auto.distance == TrajectoryManager.Distance.LONG)) {
//            if(spikeX > 1 && spikeX < 150) {
//                return propPos.LEFT;
//            } else if(spikeX > 150 && spikeX < 500) {
//                return propPos.CENTER;
//            } else {
//                return propPos.RIGHT;
//            }
//        } else {
//            if(spikeX > 100 && spikeX < 300) {
//                return propPos.LEFT;
//            } else if(spikeX > 300 && spikeX < 600) {
//                return propPos.CENTER;
//            } else {
//                return propPos.RIGHT;
//            }
//        }
//    }
//
//    public String propPosString() {
//        propPos currPos = getCurrentPropPos();
//        if(currPos == propPos.LEFT) {
//            return "PROP_LEFT";
//        } else if(currPos == propPos.CENTER) {
//            return "PROP_CENTER";
//        } else if(currPos == propPos.RIGHT) {
//            return  "PROP_RIGHT";
//        }
//        return "NOT_FOUND";
//    };
}
