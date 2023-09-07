package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DriveTrain.Mecanum;
import org.firstinspires.ftc.teamcode.DriveTrain.MotorStruct;

@TeleOp(name = "AJOPMode", group = "TeleOp")
public class AJOPMode extends LinearOpMode {
    // Def
    double drive, strafe, spin;


    @Override
    public void runOpMode() {
        Mecanum mecanum = new Mecanum();
        MotorStruct.FRMotor = hardwareMap.get(DcMotor.class, DriveConstants.kFrontRightMotor); // 0
        MotorStruct.FLMotor = hardwareMap.get(DcMotor.class, DriveConstants.kFrontLeftMotor); // 1
        MotorStruct.BRMotor = hardwareMap.get(DcMotor.class, DriveConstants.kRearRightMotor); // 2
        MotorStruct.BLMotor = hardwareMap.get(DcMotor.class, DriveConstants.kRearLeftMotor); // 3

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