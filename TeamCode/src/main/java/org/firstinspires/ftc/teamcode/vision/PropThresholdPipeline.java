package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helper.TrajectoryManager;
import org.firstinspires.ftc.teamcode.opmode.Auto;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PropThresholdPipeline extends OpenCvPipeline {

    public enum propPos {
        LEFT,
        CENTER,
        RIGHT
    }
    private static final boolean DEBUG = false;
    public static int redLeftX = (int) (350);
    public static int redLeftY = (int) (444);
    public static int redCenterX = (int) (858);
    public static int redCenterY = (int) (409);
    public static int blueLeftX = (int) (48);
    public static int blueLeftY = (int) (453);
    public static int blueCenterX = (int) (582);
    public static int blueCenterY = (int) (418);
    public static int leftWidth = (int) (200);
    public static int leftHeight = (int) (100);
    public static int centerWidth = (int) (260);
    public static int centerHeight = (int) (150);
    public static double BLUE_THRESHOLD = 70;
    public static double RED_THRESHOLD = 150;
    public double leftColor = 0.0;
    public double centerColor = 0.0;
    public Scalar left = new Scalar(0, 0, 0);
    public Scalar center = new Scalar(0, 0, 0);
    Telemetry telemetry;
    private String location = "RIGHT";

    public PropThresholdPipeline() {
        this(null);
    }

    public PropThresholdPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Rect leftZoneArea;
        Rect centerZoneArea;



        if (Auto.alliance == TrajectoryManager.Alliance.RED && Auto.distance == TrajectoryManager.Distance.LONG || Auto.alliance == TrajectoryManager.Alliance.BLUE && Auto.distance == TrajectoryManager.Distance.SHORT) {
            leftZoneArea = new Rect(redLeftX, redLeftY, leftWidth, leftHeight);
            centerZoneArea = new Rect(redCenterX, redCenterY, centerWidth, centerHeight);
        } else {
            leftZoneArea = new Rect(blueLeftX, blueLeftY, leftWidth, leftHeight);
            centerZoneArea = new Rect(blueCenterX, blueCenterY, centerWidth, centerHeight);
        }

        Mat leftZone = input.submat(leftZoneArea);
        Mat centerZone = input.submat(centerZoneArea);


        if (DEBUG) {
            Imgproc.rectangle(input, leftZoneArea, new Scalar(255, 255, 255), 2);
            Imgproc.rectangle(input, centerZoneArea, new Scalar(255, 255, 255), 2);
        }

        Imgproc.blur(leftZone, leftZone, new Size(5, 5));
        Imgproc.blur(centerZone, centerZone, new Size(5, 5));

        // Find the average color of the left and center zones
        left = Core.mean(leftZone);
        center = Core.mean(centerZone);

        double threshold = Auto.alliance == TrajectoryManager.Alliance.RED ? RED_THRESHOLD : BLUE_THRESHOLD; //RED_THRESHOLD; //ALLIANCE == Location.RED ? RED_TRESHOLD : BLUE_TRESHOLD;
        int idx = Auto.alliance == TrajectoryManager.Alliance.RED ? 0 : 2;

        leftColor = left.val[idx];
        centerColor = center.val[idx];

        if (leftColor > threshold && (left.val[0] + left.val[1] + left.val[2] - left.val[idx] - threshold < left.val[idx])) {
            // left zone has it
            location = "LEFT";
            Imgproc.rectangle(input, leftZoneArea, new Scalar(0, 255, 0), 2);
        } else if (centerColor > threshold && (center.val[0] + center.val[1] + center.val[2] - center.val[idx] - threshold < center.val[idx])) {
            // center zone has
            location = "CENTER";
            Imgproc.rectangle(input, centerZoneArea, new Scalar(0, 255, 0), 2);
        } else {
            // right zone has it
            location = "RIGHT";
        }

        if (DEBUG) {
            telemetry.addData("leftColor", left.toString());
            telemetry.addData("centerColor", center.toString());
            telemetry.addData("analysis", location.toString());

            telemetry.addData("CenterColor", center.val[idx]);
            telemetry.update();
        }

        leftZone.release();
        centerZone.release();

        return input;
    }

    public propPos getCurrentPropPos() {
        switch (location) {
            case "LEFT":
                return propPos.LEFT;
            case "CENTER":
                return propPos.CENTER;
            case "RIGHT":
                return propPos.RIGHT;
        }
        return propPos.RIGHT;
    }
}