package com.example.goldendragontesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class GDtest {
    public static void main(String[] args) {

        //System.setProperty("sun.java2d.opengl", "true");

//                        .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-70, -40, Math.toRadians(90)))
//                        .strafeRight(35)
//                        .splineToConstantHeading(new Vector2d(-25, 50), Math.toRadians(120))

        MeepMeep meepMeep = new MeepMeep(600);
        meepMeep.setDarkMode(true);
        meepMeep.getWindowFrame().setVisible(true);
        double strafeLength = 15.0;
        RoadRunnerBotEntity Blue = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.83)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-35, -60, Math.toRadians(90)))
                        // Drive to Spike Center
                        .lineToLinearHeading(new Pose2d(-35, -30, Math.toRadians(0)))

                        .lineToLinearHeading(new Pose2d(-60, -11, Math.toRadians(180)))
                        .lineToConstantHeading(new Vector2d(10, -11))
                        .addDisplacementMarker(() -> {
                            System.out.println("Run ArmSequence");
                        })
                        .splineToConstantHeading(new Vector2d(47, -35), Math.toRadians(270))
                        .lineToConstantHeading(new Vector2d(52, -15))
                        .lineToConstantHeading(new Vector2d(10, -10))
                                .lineToLinearHeading(new Pose2d(-60, -11, Math.toRadians(185)))
                        //
//                        .lineToConstantHeading(new Vector2d(10, -10))
//                        .splineToConstantHeading(new Vector2d(47, -35), Math.toRadians(270))
//                        .lineToLinearHeading(new Pose2d(52, -25, Math.toRadians(180)))
//                        .lineToLinearHeading(new Pose2d(48, -12, Math.toRadians(180)))
                        .build()
                );

        Image img = null;

        try {
            img = ImageIO.read(new File(ImageConst.FIELD_CENTERSTAGE_DARK));

        } catch (IOException e) {
        }

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(Blue)
                .start();
    }

    private static TrajectorySequence getTrajectory(String preset, DriveShim mecDrive) {
        int startingRotation = 0; // in degrees
        int startingY = 62; // This is flipped across the x-axis for red.
        int farX = -37;
        int shortX = 15;
        boolean isAllianceBlue = preset.contains("BLUE");
        int sign = isAllianceBlue ? 1 : -1; // Sign to use for Y values. If the alliance is red, the trajectory is reflected across the x-axis.
        if (preset.contains("LD")) {
            return mecDrive.trajectorySequenceBuilder(new Pose2d())
                           .splineTo(new Vector2d(15, 15), Math.toRadians(90))
                           .splineTo(new Vector2d(0, 30), Math.toRadians(180))
                           .splineTo(new Vector2d(-15, 15), Math.toRadians(270))
                           .splineTo(new Vector2d(0, 0), 0)
                           .build();
        } else { // If distance is short.
            return mecDrive.trajectorySequenceBuilder(new Pose2d(shortX, startingY * sign, Math.toRadians(startingRotation)))
                           .forward(35)
                           .build();
        }
    }
}