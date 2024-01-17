package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class DipperSubsystem extends SubsystemBase {

<<<<<<< Updated upstream
    private final static double rightLoadingPos = 0.195;
    private final static double leftLoadingPos = 0.95;
    private final static double rightScoringPos = 0.235;
    private final static double leftScoringPos = 0.973;
    private static double leftDestinationPos = 0.0;
    private static double rightDestinationPos = 0.0;
    private final Servo right, left;

=======
    public static double leftDipperServoLoadingPos = 0.195;

    public static double leftDipperServoScoringPos = 0.95;

    public static double rightDipperServoLoadingPos = 0.235;

    public static double rightDipperServoScoringPos = 0.973;


    public Servo rightDipperServo, leftDipperServo;
>>>>>>> Stashed changes

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
    }

    public void setDipperPosition(DipperPositions pos) {
<<<<<<< Updated upstream

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
=======
        if(pos == DipperPositions.LOADING_POSITION) {
            rightDipperServo.setPosition(rightDipperServoLoadingPos + 0.025); // 0.08 offset
            leftDipperServo.setPosition(leftDipperServoLoadingPos);
        } else if(pos == DipperPositions.SCORING_POSITION) {
            rightDipperServo.setPosition(rightDipperServoScoringPos);
            leftDipperServo.setPosition(leftDipperServoScoringPos - 0.06); // - 0.06 offset
>>>>>>> Stashed changes
        }
    }
    public double getLeftPosition() {
        return this.left.getPosition();
    }
    public double getRightPosition(){
        return this.right.getPosition();
    }
}
