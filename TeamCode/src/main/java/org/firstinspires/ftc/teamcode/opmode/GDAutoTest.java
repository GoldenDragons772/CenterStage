package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rr.drive.MainMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.ArmMotorEx;
import org.firstinspires.ftc.teamcode.subsystem.BucketPivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.BucketSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DipperSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.HuskySubsystem;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.subcommand.TrajectoryFollowerCommand;

import java.security.PublicKey;

@Autonomous(name = "GDAutoTest", group = "Auto")
public class GDAutoTest extends LinearOpMode {

    private MecanumDriveSubsystem drive;
    private TrajectoryFollowerCommand follower;

    private TrajectoryFollowerCommand driveToBackdrop;

    private HuskySubsystem husky;

    private HuskySubsystem.SpikeLocation currentSpikeLocation;

    private BucketSubsystem bucket;

    private BucketPivotSubsystem bucketPivot;

    private DipperSubsystem dipper;

    private ArmMotorEx armMotor;

    private String spikePos = "ERROR_404";

    private String autoName;

    Pose2d LD_RED_STARTPOS = new Pose2d(-37, -62, Math.toRadians(180));

    Pose2d SD_RED_STARTPOS = new Pose2d(15, -62, Math.toRadians(270));

    Pose2d LD_BLUE_STARTPOS = new Pose2d(-37, 62, Math.toRadians(90));

    Pose2d SD_BLUE_STARTPOS = new Pose2d(15, 62, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {

        CommandScheduler.getInstance().reset();

        drive = new MecanumDriveSubsystem(new MainMecanumDrive(hardwareMap), false);

        husky = new HuskySubsystem(hardwareMap);

        bucket = new BucketSubsystem(hardwareMap);

        bucketPivot = new BucketPivotSubsystem(hardwareMap);

        dipper = new DipperSubsystem(hardwareMap);

        armMotor = new ArmMotorEx(hardwareMap);

        Trajectory LD_RED_FOLLOW = drive.trajectoryBuilder(LD_RED_STARTPOS)
                .strafeRight(45)
                .splineToConstantHeading(new Vector2d(10, -10), Math.toRadians(0))
                .build();

        Trajectory LD_RED_BACKBOARD = drive.trajectoryBuilder(LD_RED_FOLLOW.end())
                .lineTo(new Vector2d(48, -10))
                .build();


        //Trajectory SD_RED_FOLLOW = drive.trajectoryBuilder(SD_RED_STARTPOS)


        Trajectory LD_BLUE_FOLLOW;

        Trajectory SD_BLUE_FOLLOW;



//
//        follower = new TrajectoryFollowerCommand(drive, traj);

        // Set Algorithm to Object Tracking
        husky.setAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);

        follower = new TrajectoryFollowerCommand(drive, LD_RED_FOLLOW);

        driveToBackdrop = new TrajectoryFollowerCommand(drive, LD_RED_BACKBOARD);

        drive.setPoseEstimate(LD_RED_STARTPOS);


        while(opModeInInit()) {
            currentSpikeLocation = husky.getSpikeLocation();

            switch(currentSpikeLocation) {
                case LEFT_POSITION:
                    spikePos = "LEFT_POSITION";
                    break;
                case RIGHT_POSITION:
                    spikePos = "RIGHT_POSITION";
                    break;
                case CENTER_POSITION:
                    spikePos = "CENTER_POSITION";
                    break;
            }

            // Auto Selector
            if (gamepad1.dpad_right) { // Long Distance Red Auto
                drive.setPoseEstimate(LD_RED_STARTPOS);
                follower = new TrajectoryFollowerCommand(drive, LD_RED_FOLLOW);
                autoName = "LD_RED";
            } else if (gamepad1.dpad_left) { // Short Distance Red Auto
                drive.setPoseEstimate(SD_RED_STARTPOS);
                //follower = new TrajectoryFollowerCommand(drive, SD_RED_FOLLOW);
                autoName = "SD_RED";
            } else if (gamepad1.dpad_up) { // Long Distance Blue Auto
                drive.setPoseEstimate(LD_BLUE_STARTPOS);
                //follower = new TrajectoryFollowerCommand(drive, LD_BLUE_FOLLOW);
                autoName = "LD_BLUE";
            } else if (gamepad1.dpad_down) { // Short Distance Blue Auto
                drive.setPoseEstimate(SD_BLUE_STARTPOS);
                //follower = new TrajectoryFollowerCommand(drive, SD_BLUE_FOLLOW);
                autoName = "SD_BLUE";
            }

            telemetry.addData("CurrentSpike Location", spikePos);
            telemetry.addData("Current Auto", autoName);
        }

        waitForStart();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                    follower,
                    new InstantCommand(() -> {
                        armMotor.setArmToPos(1500);
                        dipper.setDipperPosition(DipperSubsystem.DipperPositions.SCORING_POSITION);
                        bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
                    }),
                    driveToBackdrop
                ).alongWith(new RunCommand(() -> {
                    drive.update();
                }))
        );

        while(opModeIsActive()) {
            CommandScheduler.getInstance().run();
        }
    }
}
