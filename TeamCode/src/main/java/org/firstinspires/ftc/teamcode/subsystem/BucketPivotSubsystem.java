package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class BucketPivotSubsystem extends SubsystemBase {

    public static double LeftBucketPivotDroppingPos = 0.985;
    public static double LeftBucketPivotLoadingPos = 0.02;

    public static double RightBucketPivotDroppingPos = 0.61;
    public static double RightBucketPivotLoadingPos = 0.02;


    public enum BucketPivotPos {
        LOADING_POS,
        DROPPING_POS,
    }

    Servo RightBucketPivot, LeftBucketPivot;

    public BucketPivotSubsystem(HardwareMap hw) {
        RightBucketPivot = hw.get(Servo.class, "RBPivot");

        RightBucketPivot.setDirection(Servo.Direction.REVERSE);

        LeftBucketPivot = hw.get(Servo.class, "LBPivot");

        // Set Initial Pos
        RightBucketPivot.setPosition(RightBucketPivotLoadingPos);
        LeftBucketPivot.setPosition(LeftBucketPivotLoadingPos);
    }

    public void runBucketPos(BucketPivotPos pos) {
        if(pos == BucketPivotPos.DROPPING_POS) {
            RightBucketPivot.setPosition(RightBucketPivotDroppingPos);
            LeftBucketPivot.setPosition(LeftBucketPivotDroppingPos);
        } else if(pos == BucketPivotPos.LOADING_POS) {
            LeftBucketPivot.setPosition(LeftBucketPivotLoadingPos);
            RightBucketPivot.setPosition(RightBucketPivotLoadingPos);
        }
    }
}
