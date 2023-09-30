package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest", group = "Test")
public class ServoTest extends LinearOpMode {
    public void runOpMode() {
        CRServo Servo1 = hardwareMap.crservo.get("Servo1");
        CRServo Servo2 = hardwareMap.crservo.get("Servo2");
        waitForStart();

        while(opModeIsActive()) {
            Servo2.setPower(gamepad1.right_stick_y);
            Servo1.setPower(-gamepad1.right_stick_y);
        }
    }
}
