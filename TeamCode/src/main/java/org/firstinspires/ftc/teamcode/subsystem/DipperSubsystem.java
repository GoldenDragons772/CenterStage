package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class DipperSubsystem extends SubsystemBase {

    public static double rightDipperServoLoadingPos = -0.2;

    public static double leftDipperServoLoadingPos = 0.875;

    public static double rightDipperServoScoringPos = 0.855;

    public static double leftDipperServoScoringPos = -1;


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
            rightDipperServo.setPosition(0.992);
            leftDipperServo.setPosition(0.872);
        } else if(pos == DipperPositions.SCORING_POSITION) {
            rightDipperServo.setPosition(0.16);
            leftDipperServo.setPosition(-1);
        }
    }
}
