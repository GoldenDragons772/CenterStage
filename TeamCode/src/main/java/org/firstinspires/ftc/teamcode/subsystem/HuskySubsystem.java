package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HuskySubsystem extends SubsystemBase {

    public enum SpikeLocation {
        LEFT_POSITION,
        CENTER_POSITION,
        RIGHT_POSITION
    }

    private HuskyLens husky;

    private HuskyLens.Block[] blocks;

    public HuskySubsystem(HardwareMap hw) {
        husky = hw.get(HuskyLens.class, "husky");
    }

    public void setAlgorithm(HuskyLens.Algorithm alg) {
        husky.selectAlgorithm(alg);
    }

    public SpikeLocation getSpikeLocation() {
        blocks = husky.blocks();

        if(blocks.length != 0) {
            HuskyLens.Block spikeBlock = blocks[0];

            if(spikeBlock.x > 105 && spikeBlock.x < 250) {
                return SpikeLocation.CENTER_POSITION;
            } else if(spikeBlock.x > 250) {
                return SpikeLocation.RIGHT_POSITION;
            } else {
                return SpikeLocation.LEFT_POSITION;
            }
        }
        return SpikeLocation.CENTER_POSITION;
    }
}
