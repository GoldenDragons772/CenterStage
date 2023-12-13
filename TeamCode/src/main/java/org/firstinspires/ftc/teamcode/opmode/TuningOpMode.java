package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;

@TeleOp(name = "GDTeleTuning", group = "TeleOp")
public class TuningOpMode extends LinearOpMode {
    CRServo bucketServo;
    CRServo Launcher;

    // Intialize Gamepad
    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;

    // Husky Lens
    private HuskyLens huskyLens;

    //ArmDriver armDriver;

    DcMotorEx leftArmMotor, rightArmMotor;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //AlignBackboard alignBackboard = new AlignBackboard(hardwareMap, drive);
        //armDriver = new ArmDriver(hardwareMap, ArmDriver.ArmPos.LOW);

        bucketServo = hardwareMap.crservo.get("Bucket");
        Launcher = hardwareMap.crservo.get("Plane");

        // Huskylens
        huskyLens = hardwareMap.get(HuskyLens.class, "husky");

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

        // Initalize the Camera
        telemetry.addData("CAM_STATE", huskyLens.knock() ? "CONNECTED" : "DISCONNECTED");

        // Set Huskylens Mode
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {

            // Square the Values to get better Control.
            double strafe = Math.pow(gamepad1.right_stick_x, 2) * Math.signum(gamepad1.right_stick_x);
            double forward = Math.pow(gamepad1.right_stick_y, 2) * Math.signum(gamepad1.right_stick_y);
            double spin = Math.pow(gamepad1.left_stick_x, 2) * Math.signum(gamepad1.left_stick_x);

            double Speedper = 1;

            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block Count", blocks.length);
            if(blocks.length != 0) {
                // Position on the Field
                String pos;
                HuskyLens.Block colorBlock = blocks[0];
                if(colorBlock.x > 105 && colorBlock.x < 250) {

                    pos = "CENTER";
                } else if(colorBlock.x > 250) {
                    pos = "RIGHT";
                } else {
                    pos = "LEFT";
                }
                telemetry.addData("POSITION", pos);
            }

            // Shoot the Planes
            if(gamepad1.y) {
                Launcher.setPower(0.3); // 7
            } else if (gamepad1.x) {
                Launcher.setPower(1);
            }

            leftArmMotor.setPower(gamepad2.right_stick_y);
            rightArmMotor.setPower(gamepad2.right_stick_y);

            telemetry.addData("GamePad2 Y", gamepad2.right_stick_y);

            telemetry.addData("LeftArmMotor", leftArmMotor.getCurrentPosition());
            telemetry.addData("RightArmMotor", rightArmMotor.getCurrentPosition());

            // Update Telemetry
            drive.update();
            //telemetry.addData("Detected Tag", detectedTag);
            //TelemetryPacket packet = new TelemetryPacket();
            telemetry.update();
        }
    }
}

