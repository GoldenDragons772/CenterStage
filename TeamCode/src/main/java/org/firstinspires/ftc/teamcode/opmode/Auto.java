package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
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
import org.firstinspires.ftc.teamcode.vision.PropThresholdPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.HashMap;
import java.util.stream.Collectors;

import static org.firstinspires.ftc.teamcode.helper.TrajectoryManager.*;


@Autonomous(name = "Auto", group = "Auto")
public class Auto extends LinearOpMode {

    private TrajectoryFollowerCommand<TrajectorySequence> follower, driveToBackdrop, driveToSpike, carrier, park;
    private PropThresholdPipeline detectProp;
    private PropThresholdPipeline.propPos currentSpikeLocation, lastSpikeLocation;

    private boolean updateAutoChnage = false;

    public static Alliance alliance;
    public static Distance distance;

    // Subsystems
    private MecanumDriveSubsystem drive;

    //    private HuskySubsystem husky; // Retired
    private BucketPivotSubsystem bucketPivot;
    private DipperSubsystem dipper;
    private ArmMotorSubsystem armMotor;
    private IntakeSubsystem intake;

    private LinkTakeSubsystem linkTake;

    // Camera Instance
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
        //husky = new HuskySubsystem(hardwareMap); // retired
        bucketPivot = new BucketPivotSubsystem(hardwareMap);
        dipper = new DipperSubsystem(hardwareMap);
        armMotor = new ArmMotorSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        linkTake = new LinkTakeSubsystem(hardwareMap);

        detectProp = new PropThresholdPipeline(telemetry);

        // Initalize
        alliance = Alliance.RED;
        distance = Distance.SHORT;

        // Trajectory Init
        spikeLoc = new Pose2d(0, 0, Math.toRadians(0));
        startPos = new Pose2d(0, 0, Math.toRadians(0));

        currentSpikeLocation = PropThresholdPipeline.propPos.RIGHT;

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "gdeye");

        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
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

            // Check if the Spike Location has changed
            if (currentSpikeLocation != lastSpikeLocation) {
                updateAutoChnage = true;
                lastSpikeLocation = currentSpikeLocation;
            }

            // Auto Selector
            if (gamepad1.dpad_left) { // Short Distance Red Auto
                alliance = Alliance.RED;
                distance = Distance.SHORT;
                curAuto = "SD_RED";
                updateAutoChnage = true;
            } else if (gamepad1.dpad_down) { // Short Distance Blue Auto
                alliance = Alliance.BLUE;
                distance = Distance.SHORT;
                curAuto = "SD_BLUE";
                updateAutoChnage = true;
            }
            else if (gamepad1.dpad_up) { // Long Distance Blue Auto
                alliance = Alliance.BLUE;
                distance = Distance.LONG;
                curAuto = "LD_BLUE";
                updateAutoChnage = true;
            } else if (gamepad1.dpad_right) { // Long Distance Red Auto
                distance = Distance.LONG;
                alliance = Alliance.RED;
                curAuto = "LD_RED";
                updateAutoChnage = true;
            }

            if(updateAutoChnage) {
                // check if Distance is Long
                if(distance == Distance.LONG) {
                    carrier = new TrajectoryFollowerCommand<>(drive, getTrajectory(alliance, distance, Type.CARRIER));
                }

                follower = new TrajectoryFollowerCommand<>(drive, getTrajectory(alliance, distance, Type.FOLLOW));
                backDropPos = getBackdropPos();

                driveToSpike = new TrajectoryFollowerCommand<>(drive, getTrajectory(alliance, distance, Type.SPIKE));
                driveToBackdrop = new TrajectoryFollowerCommand<>(drive, getTrajectory(alliance, distance, Type.BACKBOARD));

                park = new TrajectoryFollowerCommand<>(drive, getTrajectory(alliance, distance, Type.PARK));

                // Set update to false
                updateAutoChnage = false;
            }

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
            sleep(1000);
            intake.stopIntake();
        }));
        if (distance == Distance.LONG) { // TODO: create a transition from pppp (PurPle Pixel Placing) to placing on the backdrop.
            commandGroup.addCommands(carrier);
            commandGroup.addCommands(new WaitCommand(7000));
        }
        commandGroup.addCommands(new InstantCommand(() -> {
            //
            armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.BONUS);
            dipper.setDipperPosition(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
            bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
        }));
        commandGroup.addCommands(new WaitCommand(750));
        commandGroup.addCommands(driveToBackdrop);
        commandGroup.addCommands(new InstantCommand(() -> intake.specialDispenseJustForAutoPixelDispenseThing()));
        commandGroup.addCommands(new WaitCommand(1500));
        commandGroup.addCommands(new InstantCommand(() -> { // TODO: Figure out how to make this into a function.
            intake.stopIntake();
            dipper.setDipperPosition(BucketPivotSubsystem.BucketPivotPos.LOADING_POS);
            armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.HOME);
            linkTake.setLinkTakePos(LinkTakeSubsystem.LinkPosition.HOME);
            int timeout = 1200;
            int epsilon = 550; // Machine epsilon
//            while (!(-epsilon < armMotor.getAvgArmPosition() && armMotor.getAvgArmPosition() < epsilon)) {
//                try {
//                    Thread.sleep(20);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
//            }
            bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.LOADING_POS);
        }));
        commandGroup.addCommands(park);
        return commandGroup;
    }


    private Pose2d getBackdropPos() {
            // Short Distance
            switch (currentSpikeLocation) {
                case LEFT: {
                    if(distance == Distance.SHORT) {
                        return new Pose2d(54, (alliance == Alliance.BLUE) ? 41 : -27, Math.toRadians(180));
                    }
                    return new Pose2d(54, (alliance == Alliance.BLUE) ? 43 : -27, Math.toRadians(180));
                }
                case RIGHT: {
                    if (distance == Distance.SHORT) {
                        return new Pose2d(54, (alliance == Alliance.BLUE) ? 27 : -40, Math.toRadians(180));
                    } else {
                        return new Pose2d(54, (alliance == Alliance.BLUE) ? 28 : -39, Math.toRadians(180));
                    }
                }
                case CENTER: {
                    if (distance == Distance.SHORT) {
                        return new Pose2d(54, (alliance == Alliance.BLUE) ? 35 : -34, Math.toRadians(175));
                    } else {
                        return new Pose2d(54, (alliance == Alliance.BLUE) ? 37 : -37, Math.toRadians(185));
                    }
                }
            }
        return null;
    }

    private TrajectorySequence getTrajectory(Alliance alliance, Distance distance, Type type) {
        HashMap<String, TrajectorySequence> choices = new HashMap<>();
        int reflection = alliance == Alliance.RED ? 1 : -1;
        int heading = alliance == Alliance.RED ? 90 : 270;
        spikeLoc = getSpikeLocation(alliance, distance, currentSpikeLocation);
        startPos = getStartPosition(alliance, distance);

        // Short Distance Auto

        TrajectorySequence SD_FOLLOW = drive.trajectorySequenceBuilder(new Pose2d(startPos.getX(), startPos.getY(), Math.toRadians(heading)))
                .forward(30.0)
                .build();

        TrajectorySequence SD_SPIKE = drive.trajectorySequenceBuilder(SD_FOLLOW.end())
                .lineToLinearHeading(spikeLoc)
                .forward(1)
                .build();

        TrajectorySequence SD_BACKBOARD = drive.trajectorySequenceBuilder(SD_SPIKE.end())
                .lineToLinearHeading(backDropPos)
                .build();

        TrajectorySequence SD_PARK = drive.trajectorySequenceBuilder(SD_BACKBOARD.end())
                .forward(2)
                //.lineToConstantHeading(new Vector2d(52, -10 * reflection)) // Mid
                .lineToConstantHeading(new Vector2d(50, -60 * reflection)) // Corner
                .build();

        // Long Distance Auto
        TrajectorySequence LD_FOLLOW = drive.trajectorySequenceBuilder(new Pose2d(startPos.getX(), startPos.getY(), Math.toRadians(heading)))
                .forward(30.0)
                .build();

        TrajectorySequence LD_SPIKE = drive.trajectorySequenceBuilder(LD_FOLLOW.end())
                .lineToLinearHeading(spikeLoc)
                .forward(1)
                .build();

        TrajectorySequence LD_CARRIER = drive.trajectorySequenceBuilder(LD_SPIKE.end())
                .back(10)
                .lineToLinearHeading(new Pose2d(-35, -12 * reflection, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(23, -12 * reflection, Math.toRadians(180)))
                .build();

        TrajectorySequence LD_BACKBOARD = drive.trajectorySequenceBuilder(LD_CARRIER.end())
                .setConstraints(new AngularVelocityConstraint(30), new ProfileAccelerationConstraint(30))
                //.splineToConstantHeading(new Vector2d(47, -35 * reflection), Math.toRadians(270 * reflection))
                .lineToLinearHeading(new Pose2d(37, -12 * reflection, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(37, -39 * reflection, Math.toRadians(180)))
                .lineToLinearHeading(backDropPos)
                .build();

        TrajectorySequence LD_PARK = drive.trajectorySequenceBuilder(LD_BACKBOARD.end())
                .forward(2)
                .lineToConstantHeading(new Vector2d(52, -10 * reflection)) // Left
                .build();


        // Options for Short Distance
        choices.put("SD_FOLLOW", SD_FOLLOW);
        choices.put("SD_SPIKE", SD_SPIKE);
        choices.put("SD_BACKBOARD", SD_BACKBOARD);
        choices.put("SD_PARK", SD_PARK);

        // Options for Long Distance
        choices.put("LD_FOLLOW", LD_FOLLOW);
        choices.put("LD_SPIKE", LD_SPIKE);
        choices.put("LD_CARRIER", LD_CARRIER);
        choices.put("LD_BACKBOARD", LD_BACKBOARD);
        choices.put("LD_PARK", LD_PARK);

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
            case CARRIER: {
                for(String key : choices.keySet()) {
                    if(key.contains("CARRIER")) {
                        return choices.get(key);
                    }
                }
                break;
            }
            case PARK: {
                for(String key : choices.keySet()) {
                    if(key.contains("PARK")) {
                        return choices.get(key);
                    }
                }
                break;
            }
        }
        return null; //
    }
}
