package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class TagPipeline extends OpenCvPipeline {
    boolean viewportPaused;

    private Telemetry telemetry;
    int cor_distance;
    Mat mat = new Mat();
    Mat thresh = new Mat();
    Mat img_format = new Mat();
    Mat hierarchy = new Mat();

    // Defs for the Contour...
    Rect Rectbound;
    int midpoint_x;

    @Override
    public Mat processFrame(Mat input) {


        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // Set the Color Bounds.
        Scalar lowHSV =     new Scalar(20, 100, 100); // lower bound HSV for yellow
        Scalar highHSV =    new Scalar(30, 255, 255); // higher bound HSV for yellow
        Core.inRange(mat, lowHSV, highHSV, thresh);

        // Blur the Image to get better Results.
        Imgproc.GaussianBlur(thresh, thresh, new Size(5, 5), 1);

        // Find the edges of the Images using the Canny Algorithm.
        Imgproc.Canny(thresh, img_format, 100, 200);

        // Map out Each point based on the edges.
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(img_format, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxArea = 0; // Adjust Based on Needed Area.
        int maxAreaIndex = -1;

        // Find the Largest Contour.
        for (int i = 0; i < contours.size(); i++) {
            double area = Imgproc.contourArea(contours.get(i));
            if (area > maxArea) {
                maxArea = area;
                maxAreaIndex = i;
            }
        }

        // Draw the Rectangle
        if (maxAreaIndex != -1) {
            MatOfPoint2f contourPoly = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(maxAreaIndex).toArray()), contourPoly, 3, true);
            Rectbound = Imgproc.boundingRect(new MatOfPoint(contourPoly.toArray()));


            midpoint_x = Rectbound.x + Rectbound.width / 2;
            int midpoint_y = Rectbound.y + Rectbound.height / 2;

            Imgproc.circle(mat, new Point(midpoint_x, midpoint_y), 3, new Scalar(255, 255, 255), 3);

            Imgproc.rectangle(mat, Rectbound, new Scalar(255, 255, 255), 2);

            int frame_x = input.cols() / 2;

            cor_distance = midpoint_x - frame_x;
        }

        Imgproc.rectangle(
                mat,
                new Point(
                        input.cols()/3f,
                        input.rows()/3f),
                new Point(
                        input.cols()*(2f/3f),
                        input.rows()*(2f/3f)),
                new Scalar(0, 0, 0), 1);

        Imgproc.circle(mat, new Point(input.cols() / 2f, input.rows()/2f), 2, new Scalar(0, 0, 0), 2);


        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_HSV2RGB);

        return mat; // return the mat with rectangles drawn
    }
    public int correction() {
        return this.cor_distance;
    }
}
