package org.firstinspires.ftc.teamcode.subsystem.subcommand;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystem.ArmMotorEx;

public class ArmPosCommand extends InstantCommand {
    private final ArmMotorEx arm_subsystem;

    public ArmPosCommand(ArmMotorEx subsystem, ArmMotorEx.ArmPos pos) {
        arm_subsystem = subsystem;
        addRequirements(arm_subsystem);

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
