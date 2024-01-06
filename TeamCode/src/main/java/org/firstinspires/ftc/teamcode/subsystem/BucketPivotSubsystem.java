package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BucketPivotSubsystem extends SubsystemBase {

    public enum BucketPivotPos {
        LOADING_POS,
        DROPPING_POS,
    }

    Servo RightBucketPivot, LeftBucketPivot;

    public BucketPivotSubsystem(HardwareMap hw) {
        RightBucketPivot = hw.get(Servo.class, "RBPivot");
        LeftBucketPivot = hw.get(Servo.class, "LBPivot");
    }

    public void runBucketPos(BucketPivotPos pos) {
        if(pos == BucketPivotPos.DROPPING_POS) {
            RightBucketPivot.setPosition(0.75);
            LeftBucketPivot.setPosition(0.75);
        } else if(pos == BucketPivotPos.LOADING_POS) {
            LeftBucketPivot.setPosition(0.05);
            RightBucketPivot.setPosition(0.05);
        }
    }

}
