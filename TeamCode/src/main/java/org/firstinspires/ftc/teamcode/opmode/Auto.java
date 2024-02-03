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

@Autonomous(name = "GDAutoTest", group = "Auto")
public class Auto extends LinearOpMode {

    private MecanumDriveSubsystem drive;
    private TrajectoryFollowerCommand follower;
    private TrajectoryFollowerCommand driveToBackdrop;
    private TrajectoryFollowerCommand driveToSpike;
    private TrajectoryFollowerCommand driveToCenter;
    private HuskySubsystem husky;
    private HuskySubsystem.SpikeLocation currentSpikeLocation;
    private BucketSubsystem bucket;
    private BucketPivotSubsystem bucketPivot;
    private DipperSubsystem dipper;
    private ArmMotorSubsystem armMotor;
    private IntakeSubsystem intake;
    private String spikePos = "ERROR_404";
    private String autoName = "NOT_SELECTED";
    Pose2d LD_RED_STARTPOS = new Pose2d(-37, -62, Math.toRadians(90));
    Pose2d SD_RED_STARTPOS = new Pose2d(15, -62, Math.toRadians(90));
    Pose2d LD_BLUE_STARTPOS = new Pose2d(-37, 62, Math.toRadians(270));
    Pose2d SD_BLUE_STARTPOS = new Pose2d(15, 62, Math.toRadians(270));
    // Backdrop
    Pose2d BackDropPos = new Pose2d(0, 0, Math.toRadians(0));
    // StorePose
    StorePos storePos = new StorePos();
    PoseManager poseManager = new PoseManager();
    PoseManager.spikeLocations spikeLocation = PoseManager.spikeLocations.SD_BLUE_LEFT;

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

        while(opModeInInit()) {

            // Red Long Distance Path
            Trajectory LD_RED_FOLLOW = drive.trajectoryBuilder(LD_RED_STARTPOS)
                    .forward(30)
                    .build();

            Trajectory LD_RED_SPIKE = drive.trajectoryBuilder(LD_RED_FOLLOW.end())
                    .lineToLinearHeading(poseManager.getSpikeLocation(spikeLocation))
                    .build();

            Trajectory LD_RED_BACKBOARD = drive.trajectoryBuilder(LD_RED_FOLLOW.end())
                    .splineToLinearHeading(BackDropPos, Math.toRadians(0))
                    .build();

            // Blue Long Distance Auto Path
            Trajectory LD_BLUE_FOLLOW = drive.trajectoryBuilder(LD_BLUE_STARTPOS)
                    .forward(30)
                    .build();

            Trajectory LD_BLUE_SPIKE = drive.trajectoryBuilder(LD_BLUE_FOLLOW.end())
                    .lineToLinearHeading(poseManager.getSpikeLocation(spikeLocation))
                    .build();

            Trajectory LD_BLUE_BACKBOARD = drive.trajectoryBuilder(LD_BLUE_SPIKE.end())
                    .splineToLinearHeading(BackDropPos, Math.toRadians(0))
                    .build();


            // Blue Short Distance Auto Path
            Trajectory SD_BLUE_FOLLOW = drive.trajectoryBuilder(SD_BLUE_STARTPOS)
                    .forward(30)
                    .build();

            // Short Distance Blue Spike.
            Trajectory SD_BLUE_SPIKE = drive.trajectoryBuilder(SD_BLUE_FOLLOW.end())
                    .lineToLinearHeading(poseManager.getSpikeLocation(spikeLocation))
                    .build();

            Trajectory SD_BLUE_BACKBOARD = drive.trajectoryBuilder(SD_BLUE_SPIKE.end())
                    .lineToLinearHeading(BackDropPos)
                    .build();


            // Red Short Distance Auto Path
            Trajectory SD_RED_FOLLOW = drive.trajectoryBuilder(SD_RED_STARTPOS)
                    .forward(30)
                    .build();

            Trajectory SD_RED_SPIKE = drive.trajectoryBuilder(SD_RED_FOLLOW.end())
                    .lineToLinearHeading(poseManager.getSpikeLocation(spikeLocation))
                    .build();

            Trajectory SD_RED_BACKBOARD = drive.trajectoryBuilder(SD_RED_FOLLOW.end())
                    .lineToLinearHeading(BackDropPos)
                    .build();

            // Auto Selector
            if (gamepad1.dpad_right) { // Long Distance Red Auto
                drive.setPoseEstimate(LD_RED_STARTPOS);
                follower = new TrajectoryFollowerCommand(drive, LD_RED_FOLLOW);
                autoName = "LD_RED";
            } else if (gamepad1.dpad_left) { // Short Distance Red Auto
                drive.setPoseEstimate(SD_RED_STARTPOS);
                follower = new TrajectoryFollowerCommand(drive, SD_RED_FOLLOW);
                autoName = "SD_RED";
            } else if (gamepad1.dpad_up) { // Long Distance Blue Auto
                drive.setPoseEstimate(LD_BLUE_STARTPOS);
                follower = new TrajectoryFollowerCommand(drive, LD_BLUE_FOLLOW);
                autoName = "LD_BLUE";
            } else if (gamepad1.dpad_down) { // Short Distance Blue Auto
                drive.setPoseEstimate(SD_BLUE_STARTPOS);
                follower = new TrajectoryFollowerCommand(drive, SD_BLUE_FOLLOW);
                autoName = "SD_BLUE";
            }

            currentSpikeLocation = husky.getSpikeLocation(autoName);

            // Determine the Prop location

            if(currentSpikeLocation == HuskySubsystem.SpikeLocation.LEFT_POSITION) {
                spikePos = "LEFT_POSITION";
                if(autoName.contains("BLUE")) {
                    // Blue Backdrop Left Position
                    BackDropPos = new Pose2d(54, 41, Math.toRadians(180));
                } else {
                    // Red Backdrop Left Position
                    BackDropPos = new Pose2d(54, -27, Math.toRadians(180));
                }
                driveToBackdrop = new TrajectoryFollowerCommand(drive, SD_BLUE_BACKBOARD);
                // Check if Auto is Short Distance Blue
                if(autoName == "SD_BLUE") {
                    spikeLocation = PoseManager.spikeLocations.SD_BLUE_LEFT;
                    driveToSpike = new TrajectoryFollowerCommand(drive, SD_BLUE_SPIKE);
                } else if(autoName == "LD_BLUE") {
                    spikeLocation = PoseManager.spikeLocations.LD_BLUE_LEFT;
                    driveToSpike = new TrajectoryFollowerCommand(drive, LD_BLUE_SPIKE);
                } else if(autoName == "SD_RED") {
                    spikeLocation = PoseManager.spikeLocations.SD_RED_LEFT;
                    driveToSpike = new TrajectoryFollowerCommand(drive, SD_RED_SPIKE);
                } else if(autoName == "LD_RED") {
                    spikeLocation = PoseManager.spikeLocations.LD_RED_LEFT;
                    driveToSpike = new TrajectoryFollowerCommand(drive, LD_RED_SPIKE);
                }
            } else if(currentSpikeLocation == HuskySubsystem.SpikeLocation.RIGHT_POSITION) {
                // Right Position
                spikePos = "RIGHT_POSITION";
                if(autoName.contains("BLUE")) {
                    BackDropPos = new Pose2d(54, 27, Math.toRadians(180));
                } else {
                    BackDropPos = new Pose2d(54, -41, Math.toRadians(180));
                }
                driveToBackdrop = new TrajectoryFollowerCommand(drive, SD_BLUE_BACKBOARD);
                if(autoName == "SD_BLUE") {
                    spikeLocation = PoseManager.spikeLocations.SD_BLUE_RIGHT;
                    driveToSpike = new TrajectoryFollowerCommand(drive, SD_BLUE_SPIKE);
                } else if(autoName == "LD_BLUE") {
                    spikeLocation = PoseManager.spikeLocations.LD_BLUE_RIGHT;
                    driveToSpike = new TrajectoryFollowerCommand(drive, LD_BLUE_SPIKE);
                } else if(autoName == "SD_RED") {
                    spikeLocation = PoseManager.spikeLocations.SD_RED_RIGHT;
                    driveToSpike = new TrajectoryFollowerCommand(drive, SD_RED_SPIKE);
                } else if(autoName == "LD_RED") {
                    spikeLocation = PoseManager.spikeLocations.LD_RED_RIGHT;
                    driveToSpike = new TrajectoryFollowerCommand(drive, LD_RED_SPIKE);
                }
            } else {
                spikePos = "CENTER_POSITION";
                if(autoName.contains("BLUE")) {
                    BackDropPos = new Pose2d(54, 34, Math.toRadians(180));
                } else {
                    BackDropPos = new Pose2d(54, -34, Math.toRadians(180));
                }
                driveToBackdrop = new TrajectoryFollowerCommand(drive, SD_BLUE_BACKBOARD);
                // Check if Auto is in short Distance Blue.
                if(autoName == "SD_BLUE") {
                    spikeLocation = PoseManager.spikeLocations.SD_BLUE_CENTER;
                    driveToSpike = new TrajectoryFollowerCommand(drive, SD_BLUE_SPIKE);
                } else if(autoName == "LD_BLUE") {
                    spikeLocation = PoseManager.spikeLocations.LD_BLUE_CENTER;
                    driveToSpike = new TrajectoryFollowerCommand(drive, LD_BLUE_SPIKE);
                } else if(autoName == "SD_RED") {
                    spikeLocation = PoseManager.spikeLocations.SD_RED_CENTER;
                    driveToSpike = new TrajectoryFollowerCommand(drive, SD_RED_SPIKE);
                } else if(autoName == "LD_RED") {
                    spikeLocation = PoseManager.spikeLocations.LD_RED_CENTER;
                    driveToSpike = new TrajectoryFollowerCommand(drive, LD_RED_SPIKE);
                }
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

        while(opModeIsActive()) {
            CommandScheduler.getInstance().run();
            //drive.update();
        }
        // Store End Pose
        storePos.StorePos(drive.getPoseEstimate());
    }
}
