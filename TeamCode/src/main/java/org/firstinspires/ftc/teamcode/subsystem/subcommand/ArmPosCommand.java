package org.firstinspires.ftc.teamcode.subsystem.subcommand;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystem.ArmMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.BucketPivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DipperSubsystem;

public class ArmPosCommand extends InstantCommand {
    private final ArmMotorSubsystem armMotor;
    private final BucketPivotSubsystem bucketPivot;
    private final DipperSubsystem dipper;

    public enum aPos {
        HOME,
        LOW,
        MID,
        HIGH
    }

    public ArmPosCommand(
            ArmMotorSubsystem armMotor,
            BucketPivotSubsystem bucketPivot,
            DipperSubsystem dipper
    ) {
        this.armMotor = armMotor;
        this.bucketPivot = bucketPivot;
        this.dipper = dipper;
        addRequirements(armMotor, bucketPivot, dipper);
    }

    @Override
    public void initialize() {
        armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.HOME);
        dipper.setDipperPosition(DipperSubsystem.DipperPositions.LOADING_POSITION);
        bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.LOADING_POS);
    }

    public void setScoringPos(aPos armPos) {
        switch(armPos) {
            case LOW:
                armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.LOW);
                break;
            case MID:
                armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.MIDDLE);
                break;
            case HIGH:
                armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.HIGH);
                break;
        }
        dipper.setDipperPosition(DipperSubsystem.DipperPositions.SCORING_POSITION);
        bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
    }

    public void goHomePos() {

    }
}
