package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.rr.drive.MainMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.subsystem.subcommand.TrajectoryFollowerCommand;

@Autonomous(name = "GDAutoTest", group = "Auto")
public class Auto extends LinearOpMode {

    private MecanumDriveSubsystem drive;
    private TrajectoryFollowerCommand follower;

    private TrajectoryFollowerCommand driveToBackdrop;

    private HuskySubsystem husky;

    private HuskySubsystem.SpikeLocation currentSpikeLocation;

    private BucketSubsystem bucket;

    private BucketPivotSubsystem bucketPivot;

    private DipperSubsystem dipper;

    private ArmMotorSubsystem armMotor;

    private String spikePos = "ERROR_404";

    private String autoName;

    Pose2d LD_RED_STARTPOS = new Pose2d(-37, -62, Math.toRadians(180));

    Pose2d SD_RED_STARTPOS = new Pose2d(15, -62, Math.toRadians(270));

    Pose2d LD_BLUE_STARTPOS = new Pose2d(-37, 62, Math.toRadians(180));

    Pose2d SD_BLUE_STARTPOS = new Pose2d(15, 62, Math.toRadians(270));

    // Backdrop
    Pose2d BackDropPos = new Pose2d(0, 0, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {

        CommandScheduler.getInstance().reset();

        drive = new MecanumDriveSubsystem(new MainMecanumDrive(hardwareMap), false);

        husky = new HuskySubsystem(hardwareMap);

        bucket = new BucketSubsystem(hardwareMap);

        bucketPivot = new BucketPivotSubsystem(hardwareMap);

        dipper = new DipperSubsystem(hardwareMap);

        armMotor = new ArmMotorSubsystem(hardwareMap);


        // Set Algorithm
        husky.setAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        while(opModeInInit()) {

            // Red Long Distance Path
            Trajectory LD_RED_FOLLOW = drive.trajectoryBuilder(LD_RED_STARTPOS)
                    .strafeRight(45)
                    .splineToConstantHeading(new Vector2d(10, -10), Math.toRadians(0))
                    .build();

            Trajectory LD_RED_BACKBOARD = drive.trajectoryBuilder(LD_RED_FOLLOW.end())
                    .lineTo(new Vector2d(30, -10))
                    .splineToConstantHeading(new Vector2d(54, -25), Math.toRadians(0))
                    .build();

            // Blue Long Distance Auto Path
            Trajectory LD_BLUE_FOLLOW = drive.trajectoryBuilder(LD_BLUE_STARTPOS)
                    .strafeLeft(45)
                    .splineToConstantHeading(new Vector2d(10, 10), Math.toRadians(0))
                    .build();

            Trajectory LD_BLUE_BACKBOARD = drive.trajectoryBuilder(LD_BLUE_FOLLOW.end())
                    .lineTo(new Vector2d(30, 10))
                    .splineToConstantHeading(new Vector2d(54, 25), Math.toRadians(90))
                    .build();

            // Blue Short Distance Auto Path
            Trajectory SD_BLUE_FOLLOW = drive.trajectoryBuilder(SD_BLUE_STARTPOS)
                    .forward(5)
                    .build();

            Trajectory SD_BLUE_BACKBOARD = drive.trajectoryBuilder(SD_BLUE_FOLLOW.end())
                    .splineToLinearHeading(BackDropPos, Math.toRadians(0))
                    .build();


            // Auto Selector
            if (gamepad1.dpad_right) { // Long Distance Red Auto
                drive.setPoseEstimate(LD_RED_STARTPOS);
                follower = new TrajectoryFollowerCommand(drive, LD_RED_FOLLOW);
                driveToBackdrop = new TrajectoryFollowerCommand(drive, LD_RED_BACKBOARD);
                autoName = "LD_RED";
            } else if (gamepad1.dpad_left) { // Short Distance Red Auto
                drive.setPoseEstimate(SD_RED_STARTPOS);
                //follower = new TrajectoryFollowerCommand(drive, SD_RED_FOLLOW);
                autoName = "SD_RED";
            } else if (gamepad1.dpad_up) { // Long Distance Blue Auto
                drive.setPoseEstimate(LD_BLUE_STARTPOS);
                follower = new TrajectoryFollowerCommand(drive, LD_BLUE_FOLLOW);
                driveToBackdrop = new TrajectoryFollowerCommand(drive, LD_BLUE_BACKBOARD);
                autoName = "LD_BLUE";
            } else if (gamepad1.dpad_down) { // Short Distance Blue Auto
                drive.setPoseEstimate(SD_BLUE_STARTPOS);
                follower = new TrajectoryFollowerCommand(drive, SD_BLUE_FOLLOW);
                autoName = "SD_BLUE";
            }

            currentSpikeLocation = husky.getSpikeLocation();

            // Determine the Prop location

            if(currentSpikeLocation == HuskySubsystem.SpikeLocation.LEFT_POSITION) {
                spikePos = "LEFT_POSITION";
                BackDropPos = new Pose2d(54, 44, Math.toRadians(180));
                driveToBackdrop = new TrajectoryFollowerCommand(drive, SD_BLUE_BACKBOARD);
            } else if(currentSpikeLocation == HuskySubsystem.SpikeLocation.RIGHT_POSITION) {
                spikePos = "RIGHT_POSITION";
                BackDropPos = new Pose2d(54, 26, Math.toRadians(180));
                driveToBackdrop = new TrajectoryFollowerCommand(drive, SD_BLUE_BACKBOARD);
            } else {
                spikePos = "CENTER_POSITION";
                BackDropPos = new Pose2d(54, 34, Math.toRadians(180));
                driveToBackdrop = new TrajectoryFollowerCommand(drive, SD_BLUE_BACKBOARD);
            }

            telemetry.addData("CurrentSpike Location", spikePos);
            telemetry.addData("Current Auto", autoName);
            telemetry.addData("Prop Location", husky.getSpikeX());
            telemetry.update();
        }

        waitForStart();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                    follower,
                    new InstantCommand(() -> {
                        armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.MIDDLE);
                        dipper.setDipperPosition(DipperSubsystem.DipperPositions.SCORING_POSITION);
                        bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
                    }),
                    driveToBackdrop,
                    new InstantCommand(() -> {
                        bucket.dispensePixels();
                    }),
                    new WaitCommand(2000),
                    new InstantCommand(() -> {
                        bucket.stopBucket();
                        armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.HOME);
                        dipper.setDipperPosition(DipperSubsystem.DipperPositions.LOADING_POSITION);
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
                    })
                )
        );

        while(opModeIsActive()) {
            CommandScheduler.getInstance().run();
            drive.update();
        }
    }
}
