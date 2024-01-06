package com.example.goldendragontesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

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

        RoadRunnerBotEntity Blue = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.83)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-37, -62, Math.toRadians(0)))
                        .strafeLeft(47)
                        .splineToConstantHeading(new Vector2d(47, -26), Math.toRadians(170))
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
}