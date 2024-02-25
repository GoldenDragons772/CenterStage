package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.helper.StorePos;
import org.firstinspires.ftc.teamcode.rr.drive.MainMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.subsystem.ArmMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.BucketPivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DipperSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.subcommand.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.vision.PropDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.HashMap;
import java.util.stream.Collectors;

import static org.firstinspires.ftc.teamcode.helper.TrajectoryManager.*;

@Autonomous(name = "GDAuto", group = "Auto")
public class Auto extends LinearOpMode {

    private TrajectoryFollowerCommand<TrajectorySequence> follower;
    private TrajectoryFollowerCommand<TrajectorySequence> driveToBackdrop;
    private TrajectoryFollowerCommand<TrajectorySequence> driveToSpike;

    private PropDetectionPipeline detectProp;

    private PropDetectionPipeline.propPos currentSpikeLocation;
    public static Alliance alliance;
    public static Distance distance;

    private MecanumDriveSubsystem drive;
    private HuskySubsystem husky;
    private BucketPivotSubsystem bucketPivot;
    private DipperSubsystem dipper;
    private ArmMotorSubsystem armMotor;
    private IntakeSubsystem intake;

    private OpenCvCamera camera;

    // Trajectory Settings
    private Pose2d spikeLoc;
    private Pose2d startPos;

    // Current Auto
    private String curAuto = "404";

    // Backdrop
    Pose2d backDropPos = new Pose2d(0, 0, Math.toRadians(0));

    // StorePose
    StorePos storePos = new StorePos();

    @Override
    public void runOpMode() throws InterruptedException {

        CommandScheduler.getInstance().reset();

        drive = new MecanumDriveSubsystem(new MainMecanumDrive(hardwareMap), false);
        //husky = new HuskySubsystem(hardwareMap);
        bucketPivot = new BucketPivotSubsystem(hardwareMap);
        dipper = new DipperSubsystem(hardwareMap);
        armMotor = new ArmMotorSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        detectProp = new PropDetectionPipeline(telemetry, false);

        // Initalize
        alliance = Alliance.RED;
        distance = Distance.SHORT;

        // Trajectory Init
        spikeLoc = new Pose2d(0, 0, Math.toRadians(0));
        startPos = new Pose2d(0, 0, Math.toRadians(0));

        currentSpikeLocation = PropDetectionPipeline.propPos.RIGHT;

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "gdeye");

        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

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


        //husky.setAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        while (opModeInInit()) {

            currentSpikeLocation = detectProp.getCurrentPropPos();//husky.getSpikeLocation(alliance, distance);

            // Auto Selector
            if (gamepad1.dpad_right) { // Long Distance Red Auto
                distance = Distance.LONG;
                alliance = Alliance.RED;
                curAuto = "LD_RED";
            } else if (gamepad1.dpad_left) { // Short Distance Red Auto
                alliance = Alliance.RED;
                distance = Distance.SHORT;
                curAuto = "SD_RED";
            } else if (gamepad1.dpad_up) { // Long Distance Blue Auto
                alliance = Alliance.BLUE;
                distance = Distance.LONG;
                curAuto = "LD_BLUE";
            } else if (gamepad1.dpad_down) { // Short Distance Blue Auto
                alliance = Alliance.BLUE;
                distance = Distance.SHORT;
                curAuto = "SD_BLUE";
            }

            follower = new TrajectoryFollowerCommand<>(drive, getTrajectory(alliance, distance, Type.FOLLOW));
            backDropPos = getBackdropPos();

            driveToSpike = new TrajectoryFollowerCommand<>(drive, getTrajectory(alliance, distance, Type.SPIKE));
            driveToBackdrop = new TrajectoryFollowerCommand<>(drive, getTrajectory(alliance, distance, Type.BACKBOARD));

            telemetry.addData("CurrentSpike Location", currentSpikeLocation.toString());
            telemetry.addData("Current Auto", curAuto);
           // telemetry.addData("Prop Location", husky.getSpikeX());
            telemetry.update();
        }

        // Set starting Pos
        drive.setPoseEstimate(getStartPosition(alliance, distance));

        waitForStart();

        // Stop the Camera...
        camera.stopStreaming();

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
                dipper.setDipperPosition(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
                bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
            }));
            commandGroup.addCommands(new WaitCommand(750));
            commandGroup.addCommands(driveToBackdrop);
            commandGroup.addCommands(new InstantCommand(() -> intake.specialDispenseJustForAutoPixelDispenseThing()));
            commandGroup.addCommands(new WaitCommand(2000));
            commandGroup.addCommands(new InstantCommand(() -> { // TODO: Figure out how to make this into a function.
                intake.stopIntake();
                dipper.setDipperPosition(BucketPivotSubsystem.BucketPivotPos.LOADING_POS);
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
        } else {

        }
        return commandGroup;
    }


    private Pose2d getBackdropPos() {
        switch (currentSpikeLocation) {
            case LEFT: {
                return new Pose2d(55, (alliance == Alliance.BLUE) ? 41 : -27, Math.toRadians(180));
            }
            case RIGHT: {
                return new Pose2d(55, (alliance == Alliance.BLUE) ? 27 : -41, Math.toRadians(180));
            }
            case CENTER: {
                return new Pose2d(55, (alliance == Alliance.BLUE) ? 34 : -34, Math.toRadians(180));
            }
        }
        return null;
    }

    private TrajectorySequence getTrajectory(Alliance alliance, Distance distance, Type type) {
        HashMap<String, TrajectorySequence> choices = new HashMap<>();
        //int reflection = alliance == Alliance.RED ? 1 : -1;
        int heading = alliance == Alliance.RED ? 90 : 270;
        spikeLoc = getSpikeLocation(alliance, distance, currentSpikeLocation);
        startPos = getStartPosition(alliance, distance);

        TrajectorySequence LD_FOLLOW = drive.trajectorySequenceBuilder(new Pose2d(startPos.getX(), startPos.getY(), Math.toRadians(heading)))
                .forward(30.0)
                .build();

        TrajectorySequence SD_FOLLOW = drive.trajectorySequenceBuilder(new Pose2d(startPos.getX(), startPos.getY(), Math.toRadians(heading)))
                .forward(30.0)
                .build();

        TrajectorySequence SD_SPIKE = drive.trajectorySequenceBuilder(SD_FOLLOW.end())
                .lineToLinearHeading(spikeLoc)
                .forward(1)
                .build();

        TrajectorySequence LD_SPIKE = drive.trajectorySequenceBuilder(LD_FOLLOW.end())
                .lineToLinearHeading(spikeLoc)
                .forward(1)
                .build();

        TrajectorySequence SD_BACKBOARD = drive.trajectorySequenceBuilder(SD_SPIKE.end())
                .lineToLinearHeading(backDropPos)
                .build();

        TrajectorySequence LD_BACKBOARD = drive.trajectorySequenceBuilder(LD_SPIKE.end())
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
                break;
            }

            case SHORT: {
                choices.keySet().stream().filter(s -> s.contains("LD")).collect(Collectors.toList())
                        .forEach(choices.keySet()::remove);
                break;
            }
        }
        switch (type) {
            case FOLLOW: {
                for (String key : choices.keySet()) {
                    if (key.contains("FOLLOW")) {
                        return choices.get(key);
                    }
                }
                break;
            }

            case SPIKE: {
                for (String key : choices.keySet()) {
                    if (key.contains("SPIKE")) {
                        return choices.get(key);
                    }
                }
                break;
            }
            case BACKBOARD: {
                for (String key : choices.keySet()) {
                    if (key.contains("BACKBOARD")) {
                        return choices.get(key);
                    }
                }
                break;
            }

        }
        return null;
    }
}
