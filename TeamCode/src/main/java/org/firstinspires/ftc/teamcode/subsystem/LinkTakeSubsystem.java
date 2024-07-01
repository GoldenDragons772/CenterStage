package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class LinkTakeSubsystem extends SubsystemBase {

    public static LinkPosition linkPos = LinkPosition.HOME;

    public enum LinkPosition {
        HOME(0.3),
        STK0(0.43),
        STK1(0.56),
        STK2(0.6),
        STK3(0.7),
        FLOOR(0.8);

        final private double position;

        LinkPosition(double position) {
            this.position = position;
        }

        public double getPosition() {
            return this.position;
        }
    }

    public static double servoLinkHomePos = 0;

    public static double servoFloorPos = 0.7;
    public static LinkPosition position;

    Servo linkTake;
    public LinkTakeSubsystem(HardwareMap hw) {
        linkTake = hw.get(Servo.class, "LinkTake");

        // Reverse the Sero
        linkTake.setDirection(Servo.Direction.REVERSE);

        // Set the Positions
        linkTake.setPosition(0.3);
    }

    public void setLinkTakePos(LinkPosition pos) {
        linkPos = pos;
        linkTake.setPosition(pos.getPosition());
    }

    public void incrementLinkTakePos() {
        //linkTake.setPosition(linkTake.getPosition() + 0.1);
        int nextPos = linkPos.ordinal() + 1;
        if (nextPos < LinkPosition.values().length) {
            setLinkTakePos(LinkPosition.values()[nextPos]);
        }
    }

    public void decrementLinkTakePos() {
        //linkTake.setPosition(linkTake.getPosition() - 0.1);
        int nextPos = linkPos.ordinal() - 1;
        if (nextPos >= 0) {
            setLinkTakePos(LinkPosition.values()[nextPos]);
        }
    }

    public void setLinkTakePosRaw(double pos) {
        linkTake.setPosition(pos);
    }
}