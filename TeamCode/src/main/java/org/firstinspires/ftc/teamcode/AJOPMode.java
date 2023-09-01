package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "AJOPMode", group = "TeleOp")
public class AJOPMode extends LinearOpMode {
    // Def
    double x;
    double y;
    DriveMotors Motors; // Motors

    @Override
    public void runOpMode() {
        Motors = new DriveMotors(); // Init Drive System.

        waitForStart();
        while(opModeIsActive()) {
            y = gamepad1.right_stick_y;
            x = gamepad1.left_stick_x;

            // Invert the motor so that the robot can Turn.
            Motors.FRMotor.setPower(-y - x);
            Motors.FLMotor.setPower(y - x);
            Motors.BRMotor.setPower(-y - x);
            Motors.BLMotor.setPower(y - x);
        }
    }
}