package org.firstinspires.ftc.teamcode.subsystem.subcommand;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDriveSubsystem;

public class TrajectoryFollowerCommand<T> extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final T trajectory;

    public TrajectoryFollowerCommand(MecanumDriveSubsystem drive, T trajectory) {
        this.drive = drive;
        this.trajectory = trajectory;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.followTrajectory(trajectory);
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }

}
