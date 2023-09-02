package org.firstinspires.ftc.teamcode.DriveTrain;

import org.firstinspires.ftc.teamcode.DriveMotors;

public class Mecanum {
    DriveMotors Motors = new DriveMotors();
    public void Drive(double xSpeed, double ySpeed, double zRotation) {
        Motors.FLMotor.setPower(xSpeed + ySpeed + zRotation);
        Motors.FRMotor.setPower(xSpeed - ySpeed - zRotation);
        Motors.BLMotor.setPower(xSpeed - ySpeed + zRotation);
        Motors.BRMotor.setPower(xSpeed + ySpeed - zRotation);
    }
}
