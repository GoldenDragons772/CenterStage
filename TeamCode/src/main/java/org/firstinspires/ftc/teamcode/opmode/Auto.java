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

    private TrajectoryFollowerCommand follower;
    private TrajectoryFollowerCommand driveToBackdrop;
    private TrajectoryFollowerCommand driveToSpike;

    private HuskySubsystem.SpikeLocation currentSpikeLocation;
    private Alliance alliance;
    private Distance distance;

    private MecanumDriveSubsystem drive;
    private HuskySubsystem husky;
    private BucketSubsystem bucket;
    private BucketPivotSubsystem bucketPivot;
    private DipperSubsystem dipper;
    private ArmMotorSubsystem armMotor;
    private IntakeSubsystem intake;

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


        husky.setAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        while (opModeInInit()) {

            // Auto Selector
            selectAuto();

            follower = new TrajectoryFollowerCommand(drive, getTrajectory(alliance, distance, Type.FOLLOW));
            drive.setPoseEstimate(getStartPosition(alliance,distance));
            currentSpikeLocation = husky.getSpikeLocation(alliance, distance);

            backDropPos = getBackdropPos();

            driveToBackdrop = new TrajectoryFollowerCommand(drive, getTrajectory(alliance, distance, Type.BACKBOARD));
            driveToSpike = new TrajectoryFollowerCommand(drive, getTrajectory(alliance, distance, Type.SPIKE));


            telemetry.addData("CurrentSpike Location", currentSpikeLocation.name());
            telemetry.addData("Current Auto", alliance.name() + " " + distance.name());
            telemetry.addData("Prop Location", husky.getSpikeX());
            telemetry.update();
        }

        waitForStart();

        CommandScheduler.getInstance().schedule(createCommandGroup());

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
        }
        // Store End Pose
        storePos.StorePos(drive.getPoseEstimate());
    }

    private SequentialCommandGroup createCommandGroup() {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        commandGroup.addCommands(follower);
        commandGroup.addCommands(driveToSpike);
        commandGroup.addCommands(new InstantCommand(() -> {
            intake.spikePixel();
            sleep(500);
            intake.stopIntake();
        }));
        if (distance == Distance.SHORT) { // TODO: create a transition from pppp (PurPle Pixel Placing) to placing on the backdrop.
            commandGroup.addCommands(new InstantCommand(() -> {
                armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.LOW);
                dipper.setDipperPosition(DipperSubsystem.DipperPositions.SCORING_POSITION);
                bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
            }));
            commandGroup.addCommands(new WaitCommand(750));
            commandGroup.addCommands(driveToBackdrop);
            commandGroup.addCommands(new InstantCommand(() -> bucket.dispensePixels()));
            commandGroup.addCommands(new WaitCommand(2000));
            commandGroup.addCommands(new InstantCommand(() -> { // TODO: Figure out how to make this into a function.
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
            }));
        }
        return commandGroup;
    }

    private void selectAuto() {
        if (gamepad1.dpad_right) { // Long Distance Red Auto
            distance = Distance.LONG;
            alliance = Alliance.RED;
        } else if (gamepad1.dpad_left) { // Short Distance Red Auto
            alliance = Alliance.RED;
            distance = Distance.SHORT;
        } else if (gamepad1.dpad_up) { // Long Distance Blue Auto
            alliance = Alliance.BLUE;
            distance = Distance.LONG;
        } else if (gamepad1.dpad_down) { // Short Distance Blue Auto
            alliance = Alliance.BLUE;
            distance = Distance.SHORT;
        }

    }

    private Pose2d getBackdropPos() {
        switch (currentSpikeLocation) {
            case LEFT: {
                return new Pose2d(54, (alliance == Alliance.BLUE) ? 41 : -27, Math.toRadians(180));
            }
            case RIGHT: {
                return new Pose2d(54, (alliance == Alliance.BLUE) ? 27 : -41, Math.toRadians(180));
            }
            case CENTER: {
                return new Pose2d(54, (alliance == Alliance.BLUE) ? 34 : -34, Math.toRadians(180));
            }
        }
        return null;
    }

    private Trajectory getTrajectory(Alliance alliance, Distance distance, Type type) {
        HashMap<String, Trajectory> choices = new HashMap<>();
        int reflection = alliance == Alliance.RED ? -1 : 1;
        Pose2d spikeLoc = getSpikeLocation(alliance, distance, currentSpikeLocation);
        Pose2d startPos = getStartPosition(alliance, distance);
        Trajectory LD_FOLLOW = drive.trajectoryBuilder(new Pose2d(startPos.getX(), startPos.getY() * reflection))
                .forward(30)
                .build();
        Trajectory SD_FOLLOW = drive.trajectoryBuilder(new Pose2d(startPos.getX(), startPos.getY() * reflection))
                .forward(30.0)
                .build();
        Trajectory SD_SPIKE = drive.trajectoryBuilder(SD_FOLLOW.end())
                .lineToLinearHeading(spikeLoc)
                .build();
        Trajectory LD_SPIKE = drive.trajectoryBuilder(LD_FOLLOW.end())
                .lineToLinearHeading(spikeLoc)
                .build();
        Trajectory SD_BACKBOARD = drive.trajectoryBuilder(SD_FOLLOW.end())
                .lineToLinearHeading(backDropPos)
                .build();
        Trajectory LD_BACKBOARD = drive.trajectoryBuilder(LD_SPIKE.end())
                .splineToLinearHeading(backDropPos, Math.toRadians(0.0))
                .build();

        choices.put("LD_FOLLOW", LD_FOLLOW);
        choices.put("SD_FOLLOW", SD_FOLLOW);
        choices.put("SD_SPIKE", SD_SPIKE);
        choices.put("LD_SPIKE", LD_SPIKE);
        choices.put("SD_BACKBOARD", SD_BACKBOARD);
        choices.put("LD_BACKBOARD", LD_BACKBOARD);
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
