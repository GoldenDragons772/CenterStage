package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.rr.drive.MainMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.subcommand.TrajectoryFollowerCommand;

public class GDAutoTest extends CommandOpMode {

    private MecanumDriveSubsystem drive;
    private TrajectoryFollowerCommand follower;

    Pose2d LD_RED_STARTPOS = new Pose2d(-37, -62, Math.toRadians(0));
    Pose2d LD_BLUE_STARTPOS = new Pose2d(-37, 62, Math.toRadians(0));

    Pose2d SD_RED_STARTPOS = new Pose2d(15, -60, Math.toRadians(0));

    Pose2d SD_BLUE_STARTPOS = new Pose2d(0, 0, Math.toRadians(0));

    @Override
    public void initialize() {
        drive = new MecanumDriveSubsystem(new MainMecanumDrive(hardwareMap), false);

        Trajectory traj = drive.trajectoryBuilder(LD_BLUE_STARTPOS)
                .strafeRight(35)
                .splineToConstantHeading(new Vector2d(28, 23), Math.toRadians(120))
                .build();

        follower = new TrajectoryFollowerCommand(drive, traj);

        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                follower
            )
        );
    }
}
