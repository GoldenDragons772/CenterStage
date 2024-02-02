package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.rr.drive.MainMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.subsystem.subcommand.TrajectoryFollowerCommand;

import java.util.HashMap;
import java.util.stream.Collectors;

import static org.firstinspires.ftc.teamcode.opmode.AutoPresets.*;

@Autonomous(name = "GDAutoTest", group = "Auto")
public class Auto extends LinearOpMode {

    private MecanumDriveSubsystem drive;
    private TrajectoryFollowerCommand follower;

    private TrajectoryFollowerCommand driveToBackdrop;

    private TrajectoryFollowerCommand driveToSpike;

    private HuskySubsystem husky;

    private HuskySubsystem.SpikeLocation currentSpikeLocation;

    private BucketSubsystem bucket;

    private BucketPivotSubsystem bucketPivot;

    private DipperSubsystem dipper;

    private ArmMotorSubsystem armMotor;

    private IntakeSubsystem intake;

    private Alliance alliance;
    private Distance distance;

    // Backdrop
    Pose2d backDropPos = new Pose2d(0, 0, Math.toRadians(0));

    // StorePose
    StorePos storePos = new StorePos();

    @Override
    public void runOpMode() throws InterruptedException {

        CommandScheduler.getInstance().reset();

        drive = new MecanumDriveSubsystem(new MainMecanumDrive(hardwareMap), false);

        husky = new HuskySubsystem(hardwareMap);

        bucket = new BucketSubsystem(hardwareMap);

        bucketPivot = new BucketPivotSubsystem(hardwareMap);

        dipper = new DipperSubsystem(hardwareMap);

        armMotor = new ArmMotorSubsystem(hardwareMap);

        intake = new IntakeSubsystem(hardwareMap);


        // Set Algorithm
        husky.setAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        while (opModeInInit()) {

            // Auto Selector
            if (gamepad1.dpad_right) { // Long Distance Red Auto
                drive.setPoseEstimate(LD_RED_STARTPOS);
                distance = Distance.LONG;
                alliance = Alliance.RED;
                follower = new TrajectoryFollowerCommand(drive, getTrajectory(Alliance.RED, Distance.LONG, Type.FOLLOW));
            } else if (gamepad1.dpad_left) { // Short Distance Red Auto
                drive.setPoseEstimate(SD_RED_STARTPOS);
                alliance = Alliance.RED;
                distance = Distance.SHORT;
                follower = new TrajectoryFollowerCommand(drive, getTrajectory(Alliance.RED, Distance.SHORT, Type.FOLLOW));
            } else if (gamepad1.dpad_up) { // Long Distance Blue Auto
                drive.setPoseEstimate(LD_BLUE_STARTPOS);
                alliance = Alliance.BLUE;
                distance = Distance.LONG;
                follower = new TrajectoryFollowerCommand(drive, getTrajectory(Alliance.BLUE, Distance.LONG, Type.FOLLOW));
            } else if (gamepad1.dpad_down) { // Short Distance Blue Auto
                drive.setPoseEstimate(SD_BLUE_STARTPOS);
                alliance = Alliance.BLUE;
                distance = Distance.SHORT;
                follower = new TrajectoryFollowerCommand(drive, getTrajectory(Alliance.BLUE, Distance.SHORT, Type.FOLLOW));
            }
/*
x value is constant

               if (autoName.contains("BLUE")) {
                   // Blue Backdrop Left Position
                   BackDropPos = new Pose2d(54, 41, Math.toRadians(180));
               } else {
                   // Red Backdrop Left Position
                   BackDropPos = new Pose2d(54, -27, Math.toRadians(180));
               }
                  blue right position
               if(autoName.contains("BLUE")) {
                    BackDropPos = new Pose2d(54, 27, Math.toRadians(180));
                } else {
                    BackDropPos = new Pose2d(54, -41, Math.toRadians(180));
                }
                blue center position

                 if(autoName.contains("BLUE")) {
                    BackDropPos = new Pose2d(54, 34, Math.toRadians(180));
                } else {
                    BackDropPos = new Pose2d(54, -34, Math.toRadians(180));
                }
*/
            currentSpikeLocation = husky.getSpikeLocation();
            // Determine the Prop location
            switch (currentSpikeLocation) {
                case LEFT: {
                    backDropPos = new Pose2d(54, (alliance == Alliance.BLUE) ? 41 : -27, Math.toRadians(180));
                    driveToBackdrop = new TrajectoryFollowerCommand(drive, getTrajectory(alliance, distance, Type.BACKBOARD));
                }
                case RIGHT: {
                    backDropPos = new Pose2d(54, (alliance == Alliance.BLUE) ? 27 : -41, Math.toRadians(180));
                    driveToBackdrop = new TrajectoryFollowerCommand(drive, getTrajectory(alliance, distance, Type.BACKBOARD));
                }
                case CENTER: {
                    backDropPos = new Pose2d(54, (alliance == Alliance.BLUE) ? 34 : -34, Math.toRadians(180));
                    driveToBackdrop = new TrajectoryFollowerCommand(drive, getTrajectory(alliance, distance, Type.BACKBOARD));
                }
            }
            driveToSpike = new TrajectoryFollowerCommand(drive, getTrajectory(alliance, distance, Type.SPIKE));

            telemetry.addData("CurrentSpike Location", currentSpikeLocation.name());
            telemetry.addData("Current Auto", alliance.name() + " " + distance.name());
            telemetry.addData("Prop Location", husky.getSpikeX());
            telemetry.update();
        }

        waitForStart();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        follower,
                        driveToSpike,
                        new InstantCommand(() -> {
                            intake.spikePixel();
                            sleep(500);
                            intake.stopIntake();
                        }),
                        new InstantCommand(() -> {
                            armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.LOW);
                            dipper.setDipperPosition(DipperSubsystem.DipperPositions.SCORING_POSITION);
                            bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
                        }),
                        new WaitCommand(750),
                        driveToBackdrop,
                        new InstantCommand(() -> {
                            bucket.dispensePixels();
                        }),
                        new WaitCommand(2000),
                        new InstantCommand(() -> {
                            bucket.stopBucket();
                            dipper.setDipperPosition(DipperSubsystem.DipperPositions.LOADING_POSITION);
                            armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.HOME);
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

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            //drive.update();
        }
        // Store End Pose
        storePos.StorePos(drive.getPoseEstimate());
    }

    Trajectory getTrajectory(Alliance alliance, Distance distance, Type type) {
        HashMap<String, Trajectory> choices = new HashMap<>();
        switch (alliance) {
            case RED: {
                Trajectory LD_RED_FOLLOW = drive.trajectoryBuilder(LD_RED_STARTPOS)
                        .forward(30)
                        .build();
                Trajectory SD_RED_FOLLOW = drive.trajectoryBuilder(SD_RED_STARTPOS)
                        .forward(30.0)
                        .build();
                Trajectory SD_RED_SPIKE = drive.trajectoryBuilder(SD_RED_FOLLOW.end())
                        .lineToLinearHeading(getSpikeLocation(alliance, distance, currentSpikeLocation))
                        .build();
                Trajectory LD_RED_SPIKE = drive.trajectoryBuilder(LD_RED_FOLLOW.end())
                        .lineToLinearHeading(getSpikeLocation(alliance, distance, currentSpikeLocation))
                        .build();
                Trajectory SD_RED_BACKBOARD = drive.trajectoryBuilder(SD_RED_FOLLOW.end())
                        .lineToLinearHeading(backDropPos)
                        .build();
                Trajectory LD_RED_BACKBOARD = drive.trajectoryBuilder(LD_RED_SPIKE.end())
                        .splineToLinearHeading(backDropPos, Math.toRadians(0.0))
                        .build();

                choices.put("LD_RED_FOLLOW", LD_RED_FOLLOW);
                choices.put("SD_RED_FOLLOW", SD_RED_FOLLOW);
                choices.put("SD_RED_SPIKE", SD_RED_SPIKE);
                choices.put("LD_RED_SPIKE", LD_RED_SPIKE);
                choices.put("SD_RED_BACKBOARD", SD_RED_BACKBOARD);
                choices.put("LD_RED_BACKBOARD", LD_RED_BACKBOARD);
            }

            case BLUE: {


                Trajectory LD_BLUE_FOLLOW = drive.trajectoryBuilder(LD_BLUE_STARTPOS)
                        .forward(30)
                        .build();

                Trajectory SD_BLUE_FOLLOW = drive.trajectoryBuilder(SD_BLUE_STARTPOS)
                        .forward(30.0)
                        .build();

                Trajectory SD_BLUE_SPIKE =
                        drive.trajectoryBuilder(SD_BLUE_FOLLOW.end()) //                    .lineToLinearHeading(new Pose2d(32, 29, Math.toRadians(180)))
                                .lineToLinearHeading(getSpikeLocation(alliance, distance, currentSpikeLocation))
                                .build();
                Trajectory LD_BLUE_SPIKE = drive.trajectoryBuilder(LD_BLUE_FOLLOW.end())
                        .lineToLinearHeading(getSpikeLocation(alliance, distance, currentSpikeLocation))
                        .build();

                Trajectory SD_BLUE_BACKBOARD = drive.trajectoryBuilder(SD_BLUE_SPIKE.end())
                        .lineToLinearHeading(backDropPos)
                        .build();

                Trajectory LD_BLUE_BACKBOARD = drive.trajectoryBuilder(LD_BLUE_SPIKE.end())
                        .splineToLinearHeading(backDropPos, Math.toRadians(0.0))
                        .build();

                choices.put("LD_BLUE_FOLLOW", LD_BLUE_FOLLOW);
                choices.put("SD_BLUE_FOLLOW", SD_BLUE_FOLLOW);
                choices.put("SD_BLUE_SPIKE", SD_BLUE_SPIKE);
                choices.put("LD_BLUE_SPIKE", LD_BLUE_SPIKE);
                choices.put("SD_BLUE_BACKBOARD", SD_BLUE_BACKBOARD);
                choices.put("LD_BLUE_BACKBOARD", LD_BLUE_BACKBOARD);
            }
        }
        switch (distance) {
            case LONG: {
                choices.keySet().stream().filter(s -> s.contains("SD")).collect(Collectors.toList())
                        .forEach(choices.keySet()::remove);
            }

            case SHORT: {
                choices.keySet().stream().filter(s -> s.contains("LD")).collect(Collectors.toList())
                        .forEach(choices.keySet()::remove);
            }
        }
        switch (type) {
            case FOLLOW: {
                for (String key : choices.keySet()) {
                    if (key.contains("FOLLOW")) {
                        return choices.get(key);
                    }
                }
            }

            case SPIKE: {
                for (String key : choices.keySet()) {
                    if (key.contains("SPIKE")) {
                        return choices.get(key);
                    }
                }
            }
            case BACKBOARD: {
                for (String key : choices.keySet()) {
                    if (key.contains("BACKBOARD")) {
                        return choices.get(key);
                    }
                }
            }

        }
        return null;
    }
}
