package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class DipperSubsystem extends SubsystemBase {

    public static double leftDipperServoLoadingPos = 0.195;

    public static double leftDipperServoScoringPos = 0.95;

    public static double rightDipperServoLoadingPos = 0.235;

    public static double rightDipperServoScoringPos = 0.973;


    public Servo rightDipperServo, leftDipperServo;

    public enum DipperPositions {
        LOADING_POSITION,
        SCORING_POSITION
    }


    public DipperSubsystem(HardwareMap hw) {
        this.rightDipperServo = hw.get(Servo.class,"RDipper");
        this.leftDipperServo = hw.get(Servo.class, "LDipper");

        rightDipperServo.setDirection(Servo.Direction.REVERSE);
    }

    public void setDipperPosition(DipperPositions pos) {
        if(pos == DipperPositions.LOADING_POSITION) {
            rightDipperServo.setPosition(rightDipperServoLoadingPos + 0.025); // 0.08 offset
            leftDipperServo.setPosition(leftDipperServoLoadingPos);
        } else if(pos == DipperPositions.SCORING_POSITION) {
            rightDipperServo.setPosition(rightDipperServoScoringPos);
            leftDipperServo.setPosition(leftDipperServoScoringPos - 0.06); // - 0.06 offset
        }
    }
}