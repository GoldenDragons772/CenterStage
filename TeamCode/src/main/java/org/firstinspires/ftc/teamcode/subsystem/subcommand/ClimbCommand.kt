package org.firstinspires.ftc.teamcode.subsystem.subcommand

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystem.ArmMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystem.BucketPivotSubsystem
import org.firstinspires.ftc.teamcode.subsystem.DipperSubsystem

// If climb is true, the robot will climb. If climb is false, the robot will hang.
class ClimbCommand(
    private val armMotor: ArmMotorSubsystem,
    private val bucketPivot: BucketPivotSubsystem,
    private val dipper: DipperSubsystem,
    private val climb: Boolean
) : CommandBase() {

    init {
        addRequirements(armMotor, bucketPivot, dipper)
    }

    override fun initialize() {
        dipper.setDipperPosition(BucketPivotSubsystem.BucketPivotPos.LOADING_POS)
        bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS)
        armMotor.setArmToPos(if (climb) ArmMotorSubsystem.ArmPos.HIGH.position else 100)
    }

}