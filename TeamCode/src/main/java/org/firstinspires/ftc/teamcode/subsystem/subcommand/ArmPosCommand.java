package org.firstinspires.ftc.teamcode.subsystem.subcommand;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.subsystem.ArmMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.BucketPivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DipperSubsystem;

public class ArmPosCommand extends CommandBase {
    private final ArmMotorSubsystem arm_subsystem;
    private final BucketPivotSubsystem bucketPivotSubsystem;
    private final DipperSubsystem dipperSubsystem;
    private static ArmMotorSubsystem.ArmPos armPos = ArmMotorSubsystem.ArmPos.HOME;

    public ArmPosCommand(
            ArmMotorSubsystem armMotor,
            BucketPivotSubsystem bucketPivot,
            DipperSubsystem dipper
    ) {
        arm_subsystem = armMotor;
        bucketPivotSubsystem = bucketPivot;
        dipperSubsystem = dipper;
        addRequirements(arm_subsystem, bucketPivotSubsystem, dipperSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
    }
}
