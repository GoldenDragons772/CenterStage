package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BucketSubsystem extends SubsystemBase {

    private CRServo bucketServo;

    public BucketSubsystem(HardwareMap hw) {
        this.bucketServo = hw.get(CRServo.class, "Bucket");
    }

    public void runBucket() {
        bucketServo.setPower(1);
    }

}
