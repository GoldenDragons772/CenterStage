package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DriveTrain.Mecanum;

@TeleOp(name = "AJOPMode", group = "TeleOp")
public class AJOPMode extends LinearOpMode {
    // Def
    double drive, strafe, spin;

    @Override
    public void runOpMode() {
        Mecanum mecanum = new Mecanum(); // Init Drive Train.

        waitForStart();
        while(opModeIsActive()) {
            // Get Speed From User1 Controller:
            drive = gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            spin = gamepad1.right_stick_x;

            // Pass Speed to MecDrive.
            mecanum.Drive(drive, strafe, spin);
        }
    }
}