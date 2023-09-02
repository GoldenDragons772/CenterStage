package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveMotors {
    HardwareMap hardwareMap;
    DriveConstants DriveConst;

    public DcMotor FRMotor = hardwareMap.get(DcMotor.class, DriveConstants.kFrontRightMotor); // 0
    public DcMotor FLMotor = hardwareMap.get(DcMotor.class, DriveConstants.kFrontLeftMotor); // 1
    public DcMotor BRMotor = hardwareMap.get(DcMotor.class, DriveConstants.kRearRightMotor); // 2
    public DcMotor BLMotor = hardwareMap.get(DcMotor.class, DriveConstants.kRearLeftMotor); // 3
}
