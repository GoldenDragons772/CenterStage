package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class StorePos {
    public static Pose2d OdoPose = new Pose2d(new Vector2d(0, 0), Math.toRadians(0));
    public void StorePos(Pose2d Pose) {
        OdoPose = Pose;
    }
}
