package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HuskySubsystem extends SubsystemBase {

    public enum SpikeLocation {
        LEFT,
        CENTER,
        RIGHT
    }

    private HuskyLens husky;

    private HuskyLens.Block[] blocks;

    public HuskySubsystem(HardwareMap hw) {
        husky = hw.get(HuskyLens.class, "husky");
    }

    public void setAlgorithm(HuskyLens.Algorithm alg) {
        husky.selectAlgorithm(alg);
    }

    public int getSpikeX() {
        blocks = husky.blocks();

        if(blocks.length != 0) {
            HuskyLens.Block spikeBlock = blocks[0];

            return spikeBlock.x;
        }
        return 0;
    }


    public SpikeLocation getSpikeLocation() {
        int spikeBlock = getSpikeX();

        if (spikeBlock > 100 && spikeBlock < 170) {
            return SpikeLocation.LEFT;
        } else if (spikeBlock > 170 && spikeBlock < 285) {
            return SpikeLocation.CENTER;
        } else {
            return SpikeLocation.RIGHT;
        }
    }
}
