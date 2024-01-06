package com.example.goldendragontesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import jdk.incubator.vector.VectorOperators;

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

        RoadRunnerBotEntity ldRed = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.83)
                .followTrajectorySequence(drive -> getTrajectory("LD_RED", drive));

        RoadRunnerBotEntity sdRed = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.83)
                .followTrajectorySequence(drive -> getTrajectory("SD_RED", drive));

        RoadRunnerBotEntity ldBlue = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.83)
                .followTrajectorySequence(drive -> getTrajectory("LD_BLUE", drive));


        RoadRunnerBotEntity sdBlue = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.83)
                .followTrajectorySequence(drive -> getTrajectory("SD_BLUE", drive));
        Image img = null;

        try {
            img = ImageIO.read(new File(ImageConst.FIELD_CENTERSTAGE_DARK));

        } catch (IOException e) {
        }

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(ldRed).addEntity(sdRed).addEntity(ldBlue).addEntity(sdBlue)
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
            return mecDrive.trajectorySequenceBuilder(new Pose2d(farX, startingY * sign))
                           .strafeRight(30 * sign)
                           .splineToConstantHeading(new Vector2d(-23, 10 * sign), Math.toRadians(startingRotation))
                           .splineToConstantHeading(new Vector2d(60, 10 * sign), Math.toRadians(startingRotation))
                           .build();
        } else { // If distance is short.
            return mecDrive.trajectorySequenceBuilder(new Pose2d(shortX, startingY * sign, Math.toRadians(startingRotation)))
                           .forward(35)
                           .build();
        }
    }
}