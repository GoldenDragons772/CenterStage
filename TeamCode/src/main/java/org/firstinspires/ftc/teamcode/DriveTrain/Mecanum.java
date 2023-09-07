package org.firstinspires.ftc.teamcode.DriveTrain;

import org.firstinspires.ftc.teamcode.DriveMotors;

public class Mecanum {
    DriveMotors Motors = new DriveMotors();

    public void Drive(double xSpeed, double ySpeed, double zRotation) {
        MotorStruct.FLMotor.setPower(xSpeed + ySpeed + zRotation);
        MotorStruct.FRMotor.setPower(xSpeed - ySpeed - zRotation);
        MotorStruct.BLMotor.setPower(xSpeed - ySpeed + zRotation);
        MotorStruct.BRMotor.setPower(xSpeed + ySpeed - zRotation);
    }
}
