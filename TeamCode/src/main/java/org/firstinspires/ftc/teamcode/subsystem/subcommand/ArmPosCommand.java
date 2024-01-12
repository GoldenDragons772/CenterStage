package org.firstinspires.ftc.teamcode.subsystem.subcommand;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystem.ArmMotorSubsystem;

public class ArmPosCommand extends InstantCommand {
    private final ArmMotorSubsystem arm_subsystem;

    public ArmPosCommand(ArmMotorSubsystem subsystem, ArmMotorSubsystem.ArmPos pos) {
        arm_subsystem = subsystem;
        addRequirements(subsystem);

        switch(pos) {
            case BACKBOARD_LOW:
                arm_subsystem.setArmToPos(30);
                break;
            case BACKBOARD_CENTER:
                arm_subsystem.setArmToPos(50);
                break;
            case BACKBOARD_TOP:
                arm_subsystem.setArmToPos(80);
        }
    }
}
