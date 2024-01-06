package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DipperSubsystem extends SubsystemBase {

    Servo rightDipperServo, leftDipperServo;

    public enum DipperPositions {
        LOADING_POSITION,
        SCORING_POSITION
    }


    public DipperSubsystem(HardwareMap hw) {
        this.rightDipperServo = hw.get(Servo.class, "RightDipper");
        this.leftDipperServo = hw.get(Servo.class, "LeftDipper");
    }

    public void setDipperPosition(DipperPositions pos) {

        if(pos == DipperPositions.LOADING_POSITION) {
            rightDipperServo.setPosition(0.5);
            leftDipperServo.setPosition(0.5);
        } else if(pos == DipperPositions.SCORING_POSITION) {
            rightDipperServo.setPosition(0.5);
            leftDipperServo.setPosition(0.5);
        }
    }
}