package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class LinkTakeSubsystem extends SubsystemBase {

    public enum LinkPosition {
        HOME,
        FLOOR
    }

    public static double servoLinkHomePos = 0;

    public static double servoFloorPos = 0.7;

    Servo linkTake;
    public LinkTakeSubsystem(HardwareMap hw) {
        linkTake = hw.get(Servo.class, "LinkTake");

        // Reverse the Sero
        linkTake.setDirection(Servo.Direction.REVERSE);

        // Set the Positions
        linkTake.setPosition(servoLinkHomePos);
    }

    public void setArmPos(LinkPosition pos) {
        switch (pos) {
            case HOME:
                linkTake.setPosition(servoLinkHomePos);
                break;
            case FLOOR:
                linkTake.setPosition(servoFloorPos);
                break;
        }
    }

    public void setArmPosRaw(double pos) {
        linkTake.setPosition(pos);
    }
}