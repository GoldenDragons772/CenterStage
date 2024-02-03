package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.opmode.AutoPresets;

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

        if (blocks.length != 0) {
            HuskyLens.Block spikeBlock = blocks[0];

            return spikeBlock.x;
        }
        return 0;
    }


    public SpikeLocation getSpikeLocation(AutoPresets.Alliance alliance, AutoPresets.Distance distance) {
        int spikeBlock = getSpikeX();

        if ((alliance == AutoPresets.Alliance.RED && distance == AutoPresets.Distance.SHORT) || (alliance == AutoPresets.Alliance.BLUE && distance == AutoPresets.Distance.LONG)) {
            if (spikeBlock > 0 && spikeBlock < 100) {
                return SpikeLocation.LEFT;
            } else if (spikeBlock > 100 && spikeBlock < 240) {
                return SpikeLocation.CENTER;
            } else {
                return SpikeLocation.RIGHT;
            }
        } else {
            if (spikeBlock > 100 && spikeBlock < 170) {
                return SpikeLocation.LEFT;
            } else if (spikeBlock > 170 && spikeBlock < 285) {
                return SpikeLocation.CENTER;
            } else {
                return SpikeLocation.RIGHT;
            }
        }
    }
}
