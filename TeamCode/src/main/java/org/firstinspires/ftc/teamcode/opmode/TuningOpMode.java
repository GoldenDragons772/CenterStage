package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.helper.StorePos;
import org.firstinspires.ftc.teamcode.rr.drive.MainMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.subsystem.subcommand.CarriageCommand;
import org.firstinspires.ftc.teamcode.subsystem.subcommand.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.vision.PropDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "TuningOpMode", group = "Debug")
public class TuningOpMode extends CommandOpMode {

    private MecanumDriveSubsystem drive;

    private IntakeSubsystem intake;

    private ArmMotorSubsystem armMotor;


    private DipperSubsystem dipper;

    private BucketPivotSubsystem bucketPivot;

    private DroneSubsystem drone;

    private OpenCvCamera camera;

    private PropDetectionPipeline detectProp;

    GamepadEx gpad1;

    TrajectoryFollowerCommand driveToBackDropBlue;


    @Override
    public void initialize() {

        CommandScheduler.getInstance().reset();
//
//        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();

        drive = new MecanumDriveSubsystem(new MainMecanumDrive(hardwareMap), false);
        gpad1 = new GamepadEx(gamepad1);
        intake = new IntakeSubsystem(hardwareMap);
        armMotor = new ArmMotorSubsystem(hardwareMap);
        dipper = new DipperSubsystem(hardwareMap);
        bucketPivot = new BucketPivotSubsystem(hardwareMap);
        drone = new DroneSubsystem(hardwareMap);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "gdeye");

        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

        detectProp = new PropDetectionPipeline(telemetry);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(detectProp);
                FtcDashboard.getInstance().startCameraStream(camera, 100);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        // Auto Drive Cmds
        Trajectory BACKDROP_BLUE_LEFT = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(54, 41, Math.toRadians(180)))
                .build();

        driveToBackDropBlue = new TrajectoryFollowerCommand(drive, BACKDROP_BLUE_LEFT);

        // Set Initial Pose
        drive.setPoseEstimate(StorePos.OdoPose);

        // Run Intake and also Intake Pixels.
        gpad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(
                        new InstantCommand(() -> {
                            if (armMotor.getArmPos() != ArmMotorSubsystem.ArmPos.HOME) return;
                            intake.runIntake();
                        })
                )
                .whenReleased(new InstantCommand(() -> {
                    intake.stopIntake();
                }));


        // Dispense Pixels.
        gpad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(new InstantCommand(() -> {
                    intake.dispenseIntake();

                }))
                .whenReleased(new InstantCommand(() -> {
                    intake.stopIntake();
                }));

        gpad1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new CarriageCommand(armMotor, bucketPivot, dipper, ArmMotorSubsystem.ArmPos.HIGH));


        gpad1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new CarriageCommand(armMotor, bucketPivot, dipper, ArmMotorSubsystem.ArmPos.MIDDLE));

        gpad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    new CarriageCommand(armMotor, bucketPivot, dipper, ArmMotorSubsystem.ArmPos.MIDDLE).execute();
                    int timeout = 1200;
                    int epsilon = 550; // Machine epsilon
                    while (!(-epsilon < armMotor.getAvgArmPosition() && armMotor.getAvgArmPosition() < epsilon)) {
                        try {
                            Thread.sleep(20);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                    }
                    bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.LOADING_POS);
                }));

        gpad1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> {
                    drone.shootDrone();
                }));

        gpad1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> {
                    drone.loadDrone();
                }));

        // Auto Align Feature
        gpad1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(
                        driveToBackDropBlue
                );

        schedule(new RunCommand(() -> {

            double strafe = Math.pow(gamepad1.right_stick_x, 2) * Math.signum(gamepad1.right_stick_x);
            double forward = Math.pow(gamepad1.right_stick_y, 2) * Math.signum(gamepad1.right_stick_y);
            double spin = Math.pow(gamepad1.left_stick_x, 2) * Math.signum(gamepad1.left_stick_x);

            double Speedper = 1;

            drive.drive(
                    forward * Speedper,
                    strafe * Speedper,
                    spin * Speedper
            );


        }).alongWith(new RunCommand(() -> {
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("LeftArmPos", armMotor.leftArmMotor.getCurrentPosition());
            telemetry.addData("RightArmPos", armMotor.rightArmMotor.getCurrentPosition());
            telemetry.addData("PIDError", 1500 - (armMotor.leftArmMotor.getCurrentPosition() + armMotor.rightArmMotor.getCurrentPosition()) / 2);
            telemetry.addData("Correction", armMotor.correction);
            telemetry.addData("PropPos", detectProp.propPosString());

//            telemetry.addData("DipperRightServo", dipper.rightDipperServo.getPosition());
//            telemetry.addData("DipperLeftServo", dipper.leftDipperServo.getPosition());
            telemetry.update();
        })));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        try {
            camera.stopStreaming();
        } catch (Exception e) {
            // Do Nothing...
        }
    }
}
