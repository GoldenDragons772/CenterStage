package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utils.PIDControl;
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
    SampleMecanumDrive mecDrive;

    private HuskyLens huskyLens;

    // x-val for center Position on HuskyCAM
    int centerX = 160;
    private int error(int blockXValue) {
        return blockXValue - centerX;
    }

    AprilTagDetection tagOfInterest = null;
    public AlignBackboard(HardwareMap hm, SampleMecanumDrive drive) {
        hardwareMap = hm;
        mecDrive = drive;
    }

    private class AlignToTag implements Runnable {

        int TAG_OF_INTEREST = 0;
        boolean aligned = false;
        boolean tagFound = false;
        AlignToTag(int tag) {
            TAG_OF_INTEREST = tag;
        }
        @Override
        public void run() {

            // Set Algorithm for HuskyLens
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

            // Get Data from HuskyLens
            HuskyLens.Block[] blocks = huskyLens.blocks();

            // Check if any Tag is in the Frame
            if(blocks.length != 0) {
                // Get the First Tag
                HuskyLens.Block block = blocks[0];
                if(block.id == TAG_OF_INTEREST) {
                    boolean aligned = false;
                    while(!aligned) {
                        // Leave the Thread if Interrupted
                        if(Thread.currentThread().isInterrupted()) {
                            break;
                        }
                        int alError = error(block.x);

                        double alPID = -(pid.PID(alError) / 45);
                        mecDrive.setWeightedDrivePower(new Pose2d(0, alPID, 0));
                    }
                }
            }
        }
    }


    public void lockToTag(int tag) {
        // Initialize HuskyLens
        huskyLens = hardwareMap.get(HuskyLens.class, "husky");

        // Check if HuskyLens is Connected
        if(huskyLens.knock()) {
            // Detect AprilTag
            Thread detectTag = new Thread(new AlignToTag(tag));
            detectTag.start();
        }
    }
}