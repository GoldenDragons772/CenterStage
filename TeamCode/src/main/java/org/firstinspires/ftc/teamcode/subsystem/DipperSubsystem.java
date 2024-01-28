package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class DipperSubsystem extends SubsystemBase {
    public final static double rightLoadingPos = 0.235;
    public final static double leftLoadingPos = 0.195;
    public final static double rightScoringPos = 0.973;
    public final static double leftScoringPos = 0.95;
    private static double leftDestinationPos = 0.0;
    private static double rightDestinationPos = 0.0;
    private final Servo right, left;


    public enum DipperPositions {
        LOADING_POSITION,
        SCORING_POSITION
    }

    public void waitForIdle() {
        while (this.right.getPosition() != leftDestinationPos || this.left.getPosition() != leftDestinationPos){
            continue;
        }
    }

    public DipperSubsystem(HardwareMap hw) {
        this.right = hw.get(Servo.class, "RDipper");
        this.left = hw.get(Servo.class, "LDipper");

        right.setDirection(Servo.Direction.REVERSE);

        // Set Initial Loading Pos
        right.setPosition(rightLoadingPos);
        left.setPosition(leftLoadingPos);
    }

    public void setDipperPosition(DipperPositions pos) {

        if (pos == DipperPositions.LOADING_POSITION) {
            right.setPosition(rightLoadingPos + 0.025); // 0.025 offset
            left.setPosition(leftLoadingPos);
            rightDestinationPos = rightLoadingPos;
            leftDestinationPos = leftLoadingPos;
        } else if (pos == DipperPositions.SCORING_POSITION) {
            right.setPosition(rightScoringPos);
            left.setPosition(leftScoringPos - 0.06); // - 0.06 offset
            rightDestinationPos = rightScoringPos;
            leftDestinationPos = leftScoringPos;
        }
    }
    public double getLeftPosition() {
        return this.left.getPosition();
    }
    public double getRightPosition(){
        return this.right.getPosition();
    }
}
