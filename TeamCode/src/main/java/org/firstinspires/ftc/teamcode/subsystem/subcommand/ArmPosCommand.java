package org.firstinspires.ftc.teamcode.subsystem.subcommand;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.ArmMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.BucketPivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DipperSubsystem;

public class ArmPosCommand extends CommandBase {
    private final ArmMotorSubsystem armMotor;
    private final BucketPivotSubsystem bucketPivot;
    private final DipperSubsystem dipper;
    private static ArmMotorSubsystem.ArmPos armPos = ArmMotorSubsystem.ArmPos.HOME;

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
        super.initialize();
    }
}
