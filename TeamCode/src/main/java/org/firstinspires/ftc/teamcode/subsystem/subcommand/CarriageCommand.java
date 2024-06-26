package org.firstinspires.ftc.teamcode.subsystem.subcommand;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.ArmMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.BucketPivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DipperSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LinkTakeSubsystem;

import static org.firstinspires.ftc.teamcode.subsystem.BucketPivotSubsystem.BucketPivotPos.DROPPING_POS;
import static org.firstinspires.ftc.teamcode.subsystem.BucketPivotSubsystem.BucketPivotPos.LOADING_POS;

// The carriage command is used to control the arm, bucket pivot, and dipper subsystems
public class CarriageCommand extends CommandBase {
    private final ArmMotorSubsystem armMotor;
    private final BucketPivotSubsystem bucketPivot;
    private final DipperSubsystem dipper;
    private final ArmMotorSubsystem.ArmPos armPos;
    private final LinkTakeSubsystem linkTakeSubsystem;

    public CarriageCommand(
            ArmMotorSubsystem armMotor,
            BucketPivotSubsystem bucketPivot,
            DipperSubsystem dipper,
            ArmMotorSubsystem.ArmPos armPos,
            LinkTakeSubsystem linkTakeSubsystem
    ) {
        this.armMotor = armMotor;
        this.bucketPivot = bucketPivot;
        this.dipper = dipper;
        this.armPos = armPos;
        this.linkTakeSubsystem = linkTakeSubsystem;
        addRequirements(armMotor, bucketPivot, dipper, linkTakeSubsystem);
    }

    @Override
    public void initialize() {
        BucketPivotSubsystem.BucketPivotPos bucketPivotPos = armPos == ArmMotorSubsystem.ArmPos.HOME ? LOADING_POS : DROPPING_POS;
        armMotor.setArmToPos(armPos);
        bucketPivot.runBucketPos(bucketPivotPos);
        dipper.setDipperPosition(bucketPivotPos);
    }
}
