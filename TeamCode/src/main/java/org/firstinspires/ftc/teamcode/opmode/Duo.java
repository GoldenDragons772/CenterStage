package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.rr.drive.MainMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.ArmMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.BucketPivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.BucketSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DipperSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DroneSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.GamepadTrigger;

@TeleOp(name = "Duo", group = "TeleOp")
public class Duo extends CommandOpMode {

    private MecanumDriveSubsystem drive;
    private IntakeSubsystem intake;
    private BucketSubsystem bucket;
    private BucketPivotSubsystem bucketPivot;
    private DipperSubsystem dipper;
    private ArmMotorSubsystem armMotor;
    private DroneSubsystem drone;
    private GamepadEx gpad1, gpad2;
    private Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();

    @Override
    public void initialize() {

        CommandScheduler.getInstance().reset();

        drive = new MecanumDriveSubsystem(new MainMecanumDrive(hardwareMap), false);
        gpad1 = new GamepadEx(gamepad1);
        gpad2 = new GamepadEx(gamepad2);
        intake = new IntakeSubsystem(hardwareMap);
        armMotor = new ArmMotorSubsystem(hardwareMap);
        bucket = new BucketSubsystem(hardwareMap);
        dipper = new DipperSubsystem(hardwareMap);
        bucketPivot = new BucketPivotSubsystem(hardwareMap);
        drone = new DroneSubsystem(hardwareMap);


        // Run Intake and also Intake Pixels.
        gpad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
             .whenHeld(
                     new InstantCommand(() -> {
                         if (armMotor.getArmPos() != ArmMotorSubsystem.ArmPos.HOME) return;
                         intake.runIntake();
                         bucket.intakePixels();
                     })
             )
             .whenReleased(new InstantCommand(() -> {
                 intake.stopIntake();
                 bucket.stopBucket();
             }));


        // Dispense Pixels.
        gpad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
             .whenHeld(new InstantCommand(() -> {
                 bucket.dispensePixels();
                 intake.dispenseIntake();
             }))
             .whenReleased(new InstantCommand(() -> {
                 bucket.stopBucket();
                 intake.stopIntake();
             }));

        // Climb
        gpad1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    dipper.setDipperPosition(DipperSubsystem.DipperPositions.LOADING_POSITION);
                    bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
                    armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.HIGH);
                }));

        // Hang
        gpad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    dipper.setDipperPosition(DipperSubsystem.DipperPositions.LOADING_POSITION);
                    bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
                    armMotor.setArmToPos(100);
                }));

        // Middle Position
        gpad2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
             .whenPressed(new InstantCommand(() -> {
                 armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.MIDDLE);
                 dipper.setDipperPosition(DipperSubsystem.DipperPositions.SCORING_POSITION);
                 bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
             }));

        // Low Positions
        gpad2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(new InstantCommand(() -> {
                armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.LOW);
                dipper.setDipperPosition(DipperSubsystem.DipperPositions.SCORING_POSITION);
                bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
            }));


        // Top Position
        gpad2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.HIGH);
                    dipper.setDipperPosition(DipperSubsystem.DipperPositions.SCORING_POSITION);
                    bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
                }));

        // Home Position
        gpad2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    dipper.setDipperPosition(DipperSubsystem.DipperPositions.LOADING_POSITION);
                    armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.HOME);
//                    // Wait for Arm Before going to Loading Position.
//                    int timeout = 1200;
//                    int epsilon = 550; // Machine epsilon
//                    while (!(-epsilon < armMotor.getAvgArmPosition() && armMotor.getAvgArmPosition() < epsilon)) {
//                        try {
//                            Thread.sleep(20);
//                        } catch (InterruptedException e) {
//                            throw new RuntimeException(e);
//                        }
//                    }
                    bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.LOADING_POS);
                }));

        // Arm Manual Control
        gpad2.getGamepadButton(GamepadKeys.Button.X)
            .whenPressed(new InstantCommand(() -> {
                armMotor.setArmMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }));

        gpad2.getGamepadButton(GamepadKeys.Button.B)
            .whenPressed(new InstantCommand(() -> {
                armMotor.setArmMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gamepad2.rumble(1000);
            }));

        gpad1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> {
                    drone.shootDrone();
                }));

        gpad1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> {
                    drone.loadDrone();
                }));
    }

    @Override
    public void run() {
        // Run Scheduler.
        CommandScheduler.getInstance().run();

        // Drive System
        double strafe = Math.pow(gamepad1.right_stick_x, 2) * Math.signum(gamepad1.right_stick_x);
        double forward = Math.pow(gamepad1.right_stick_y, 2) * Math.signum(gamepad1.right_stick_y);
        double spin = Math.pow(gamepad1.left_stick_x, 2) * Math.signum(gamepad1.left_stick_x);

        double speedMultiplier = 1;

        drive.drive(
                forward * speedMultiplier,
                strafe * speedMultiplier,
                spin * speedMultiplier
        );

        drive.update();
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("LeftArmPos", armMotor.leftArmMotor.getCurrentPosition());
        telemetry.addData("RightArmPos", armMotor.rightArmMotor.getCurrentPosition());
        telemetry.addData("PIDError", 1500 - (armMotor.leftArmMotor.getCurrentPosition() + armMotor.rightArmMotor.getCurrentPosition()) / 2);
        telemetry.addData("Correction", armMotor.correction);
        telemetry.update();

    }
}
