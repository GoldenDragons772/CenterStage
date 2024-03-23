package org.firstinspires.ftc.teamcode.helper;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class StorePos {
    public static Pose2d OdoPose = new Pose2d(-37.0,62.0,Math.toRadians(270.0));
    public void StorePos(Pose2d Pose) {
        OdoPose = Pose;
    }
}
