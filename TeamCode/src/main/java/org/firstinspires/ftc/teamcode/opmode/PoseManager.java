package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseManager {
    public enum spikeLocations {
        SD_BLUE_LEFT,
        SD_BLUE_CENTER,
        SD_BLUE_RIGHT,
        LD_BLUE_LEFT,
        LD_BLUE_CENTER,
        LD_BLUE_RIGHT,
        SD_RED_LEFT,
        SD_RED_CENTER,
        SD_RED_RIGHT,
        LD_RED_LEFT,
        LD_RED_CENTER,
        LD_RED_RIGHT
    }

    public Pose2d getSpikeLocation(spikeLocations spikeLoc) {
        switch (spikeLoc) {
            case SD_BLUE_LEFT:
                return new Pose2d(21, 42, Math.toRadians(270));
            case SD_BLUE_CENTER:
                return new Pose2d(21, 24, Math.toRadians(180));
            case SD_BLUE_RIGHT:
                return new Pose2d(-2, 38, Math.toRadians(270));
        }
        return new Pose2d(0, 0, Math.toRadians(0));
    }
}
