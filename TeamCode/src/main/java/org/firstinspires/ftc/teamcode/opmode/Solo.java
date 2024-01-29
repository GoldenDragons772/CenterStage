package org.firstinspires.ftc.teamcode.opmode;

import androidx.core.os.TraceKt;

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
import org.firstinspires.ftc.teamcode.rr.drive.MainMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.ArmMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.BucketPivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.BucketSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DipperSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DroneSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.subcommand.TrajectoryFollowerCommand;

@TeleOp(name = "Solo", group = "TeleOp")
public class Solo extends CommandOpMode {

    private MecanumDriveSubsystem drive;

    private IntakeSubsystem intake;

    private ArmMotorSubsystem armMotor;

    private BucketSubsystem bucket;

    private DipperSubsystem dipper;

    private BucketPivotSubsystem bucketPivot;

    private DroneSubsystem drone;

    GamepadEx gpad1;

    TrajectoryFollowerCommand driveToBackDropBlue;


    @Override
    public void initialize() {
        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();

        drive = new MecanumDriveSubsystem(new MainMecanumDrive(hardwareMap), false);
        gpad1 = new GamepadEx(gamepad1);
        intake = new IntakeSubsystem(hardwareMap);
        armMotor = new ArmMotorSubsystem(hardwareMap);
        bucket = new BucketSubsystem(hardwareMap);
        dipper = new DipperSubsystem(hardwareMap);
        bucketPivot = new BucketPivotSubsystem(hardwareMap);
        drone = new DroneSubsystem(hardwareMap);

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
                         bucket.intakePixels();
                     })
             )
             .whenReleased(new InstantCommand(() -> {
                 intake.stopIntake();
                 bucket.stopBucket();
             }));


        // Dispense Pixels.
        gpad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
             .whenHeld(new InstantCommand(() -> {
                 bucket.dispensePixels();
                 intake.dispenseIntake();

             }))
             .whenReleased(new InstantCommand(() -> {
                 bucket.stopBucket();
                 intake.stopIntake();
             }));

        gpad1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
             .whenPressed(new InstantCommand(() -> {
                 armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.HIGH);
                 dipper.setDipperPosition(DipperSubsystem.DipperPositions.SCORING_POSITION);
                 bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
             }));


        gpad1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
             .whenPressed(new InstantCommand(() -> {
                 armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.MIDDLE);
                 dipper.setDipperPosition(DipperSubsystem.DipperPositions.SCORING_POSITION);
                 bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
             }));

        // Low Positions
        gpad1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(new InstantCommand(() -> {
                armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.LOW);
                dipper.setDipperPosition(DipperSubsystem.DipperPositions.SCORING_POSITION);
                bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
            }));

        gpad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
             .whenPressed(new InstantCommand(() -> {
                 dipper.setDipperPosition(DipperSubsystem.DipperPositions.LOADING_POSITION);
                 armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.HOME);
                 long startTime = System.currentTimeMillis();
                 int timeout = 1200;
                 int epsilon = 10; // Machine epsilon
//                 while (!(-epsilon < armMotor.getAvgArmPosition() && armMotor.getAvgArmPosition() < epsilon)) {
//                     if((System.currentTimeMillis() - startTime) > timeout) {
//                         break;
//                     } else {
//                         try {
//                             Thread.sleep(20);
//                         } catch (InterruptedException e) {
//                             throw new RuntimeException(e);
//                         }
//                     }
//                 }
                 bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.LOADING_POS);
                 armMotor.waitForIdle();
             }).andThen(new InstantCommand(() -> {armMotor.stopResetArm();})));

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
    }

    @Override
    public void run() {



        double strafe = Math.pow(gamepad1.right_stick_x, 2) * Math.signum(gamepad1.right_stick_x);
        double forward = Math.pow(gamepad1.right_stick_y, 2) * Math.signum(gamepad1.right_stick_y);
        double spin = Math.pow(gamepad1.left_stick_x, 2) * Math.signum(gamepad1.left_stick_x);

        double Speedper = 1;

        drive.drive(
                forward * Speedper,
                strafe * Speedper,
                spin * Speedper
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
//            telemetry.addData("DipperRightServo", dipper.rightDipperServo.getPosition());
//            telemetry.addData("DipperLeftServo", dipper.leftDipperServo.getPosition());
        telemetry.update();

        CommandScheduler.getInstance().run();
    }
}
