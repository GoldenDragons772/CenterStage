package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.Subsystem.ArmDriver;

@TeleOp(name = "GDTeleOp", group = "TeleOp")
public class GDTeleOp extends LinearOpMode {
    CRServo bucketServo;
    CRServo Launcher;

    // Intialize Gamepad
    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;

    //ArmDriver armDriver;

    DcMotorEx leftArmMotor, rightArmMotor;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //AlignBackboard alignBackboard = new AlignBackboard(hardwareMap, drive);
        //armDriver = new ArmDriver(hardwareMap, ArmDriver.ArmPos.LOW);

        bucketServo = hardwareMap.crservo.get("Bucket");
        Launcher = hardwareMap.crservo.get("Plane");

        // Intialize Motor
        leftArmMotor = hardwareMap.get(DcMotorEx.class, "LeftArmMotor");
        //leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightArmMotor = hardwareMap.get(DcMotorEx.class, "RightArmMotor");
        //rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set the Pose to the Stored Pose from Autonomous.
        drive.setPoseEstimate(StorePos.OdoPose);

        // Backboard Position
        int backBoardPos = 0;

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        waitForStart();
        while(opModeIsActive()) {

            // Square the Values to get better Control.
            double strafe = Math.pow(gamepad1.right_stick_x, 2) * Math.signum(gamepad1.right_stick_x);
            double forward = Math.pow(gamepad1.right_stick_y, 2) * Math.signum(gamepad1.right_stick_y);
            double spin = Math.pow(gamepad1.left_stick_x, 2) * Math.signum(gamepad1.left_stick_x);

            double Speedper = 1;

            // Set the Power of the Drive Train
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -forward * Speedper,
                            -strafe * Speedper,
                            -spin * Speedper
                    )
            );

//            // Align to the Backboard
//            if(gamepad1.a && backBoardPos != AlignBackboard.BB_BL_CENTER) {
//                alignBackboard.lockToTag(AlignBackboard.BB_BL_CENTER);
//                backBoardPos = AlignBackboard.BB_BL_CENTER;
//            }

            // Shoot the Planes
            if(gamepad1.y) {
                Launcher.setPower(0.3); // 7
            } else if (gamepad1.x) {
                Launcher.setPower(1);
            }

            if(gamepad1.right_trigger > 0.5) {
                leftArmMotor.setPower(gamepad1.right_trigger);
                rightArmMotor.setPower(gamepad1.right_trigger);
            } else if(gamepad1.left_trigger > 0.5) {
                leftArmMotor.setPower(-gamepad1.left_trigger);
                rightArmMotor.setPower(-gamepad1.left_trigger);
            }
//            // Make Motor Go UP
//            leftArmMotor.setPower(gamepad1.right_trigger);
//            rightArmMotor.setPower(gamepad1.right_trigger);
//
//            // Make Motor Go Down
//            leftArmMotor.setPower(-gamepad1.right_trigger);
//            rightArmMotor.setPower(-gamepad1.right_trigger);

            telemetry.addData("GamePad1 left", gamepad1.left_trigger);
            telemetry.addData("GamePad1 right", gamepad1.right_trigger);


            // Update Telemetry
            drive.update();
            //telemetry.addData("Detected Tag", detectedTag);
            TelemetryPacket packet = new TelemetryPacket();
            telemetry.update();
        }
    }
}