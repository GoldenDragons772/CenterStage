package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.rr.drive.MainMecanumDrive;
import org.firstinspires.ftc.teamcode.helper.PIDControl;
import org.openftc.apriltag.AprilTagDetection;


public class AlignBackboard {


    // Backboard Tags
    public static int BB_RD_LEFT = 1;

    public static int BB_RD_CENTER = 2;
    public static int BB_RD_RIGHT = 3;

    public static int BB_BL_LEFT = 4;

    public static int BB_BL_CENTER = 5;

    public static int BB_BL_RIGHT = 6;

    PIDControl pid = new PIDControl();
    HardwareMap hardwareMap;
    MainMecanumDrive mecDrive;

    private HuskyLens huskyLens;

    // x-val for center Position on HuskyCAM
    int centerX = 160;

    private int error(int blockXValue) {
        return blockXValue - centerX;
    }

    AprilTagDetection tagOfInterest = null;

    public AlignBackboard(HardwareMap hm, MainMecanumDrive drive) {
        hardwareMap = hm;
        mecDrive = drive;
    }

    private class AlignToTag implements Runnable {

        int TAG_OF_INTEREST = 0; // TODO: find a way to set this or ask sanjith to
        boolean aligned = false;
        boolean tagFound = false;
        double minError = 0.5; // Arbitrary value picked arbitrarily

        AlignToTag(int tag) {
            TAG_OF_INTEREST = tag;
        }

        @Override
        public void run() {

            // Set Algorithm for HuskyLens
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);


            // Iterate through tags and check if they're the tag we're interested in.
            for (HuskyLens.Block block : huskyLens.blocks()) {
                if (block.id != TAG_OF_INTEREST) {
                    tagFound = false;
                    continue;
                }

                tagFound = true;

                // While the error is greater than a given amount, continue the negative feedback loop.
                // I don't like calculating the error twice, but subtracting is handled on the ALU and any respectable CPU can handle hundreds of millions or billions of these a second.
                while (error(block.x) > minError) {
                    if (Thread.currentThread().isInterrupted()) break; // Abort if interrupted.

                    int alError = error(block.x);

                    double correction = -(pid.PID(alError) / 45);
                    mecDrive.setWeightedDrivePower(new Pose2d(0, correction, 0));
                }
                aligned = true;
            }

        }
    }


    public void lockToTag(int tag) {
        // Initialize HuskyLens
        huskyLens = hardwareMap.get(HuskyLens.class, "husky");

        // Check if HuskyLens is Connected
        if (huskyLens.knock()) {
            // Detect AprilTag
            Thread detectTag = new Thread(new AlignToTag(tag));
            detectTag.start();
        }
    }
}
