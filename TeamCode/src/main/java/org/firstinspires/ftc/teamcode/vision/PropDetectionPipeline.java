package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

public class PropDetectionPipeline extends OpenCvPipeline {

    public enum propPos {
        LEFT,
        CENTER,
        RIGHT
    }

    Telemetry telemetry;

    private int spikeX = 0;

    Mat mat = new Mat();
    Mat thresh = new Mat();

    Mat edges = new Mat();

    Mat hierarchy = new Mat();

    public PropDetectionPipeline(Telemetry telemetry) {
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
//        Scalar lowHSV = new Scalar(0, 116, 44); // lower bound HSV for Red
//        Scalar highHSV = new Scalar(179, 243, 138); // higher bound HSV for Red

        // lower bound for Blue
        Scalar lowHSV = new Scalar(104, 90, 78); // lower bound HSV for Blue
        Scalar highHSV = new Scalar(134, 241, 255); // higher bound HSV for Blue


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

        for (int i = 0; i != boundRect.length; i++) {
            // If the area of the rectangle is greater than 2000, then its a Spike
            if (boundRect[i].area() > 2000) {

                spikeX = boundRect[i].x + boundRect[i].width / 2;
                int y = boundRect[i].y + boundRect[i].height / 2;

                if(y > 150) {
                    Imgproc.rectangle(mat, boundRect[i], new Scalar(255, 255, 255), 2);

                    // Find the center of the rectangle


                    // Print the center of the rectangle
                    telemetry.addData("Prop Position", spikeX);
                    telemetry.addData("Prop Y Position", y);
                    telemetry.addData("Prop Location", propPosString());
                    telemetry.update();

                    // Draw a circle at the center of the rectangle
                    Imgproc.circle(mat, new Point(spikeX, y), 5, new Scalar(255, 255, 255), 1);
                }
            }
        }

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_HSV2RGB);

        return mat; // return the mat with rectangles drawn
    }


    public propPos getCurrentPropPos() {
        if(spikeX < 100) {
            return propPos.LEFT;
        } else if(spikeX > 150 && spikeX < 500) {
            return propPos.CENTER;
        } else {
            return propPos.RIGHT;
        }
    }

    public String propPosString() {
        propPos currPos = getCurrentPropPos();
        switch(currPos) {
            case LEFT:
                return "PROP_LEFT";
            case CENTER:
                return "PROP_CENTER";
            case RIGHT:
                return "PROP_RIGHT";
        }
        return "404";
    }
}
