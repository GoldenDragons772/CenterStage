package org.firstinspires.ftc.teamcode.opmode;

public enum AutoPresets {
    LD_BLUE(0),
    SD_BLUE(1),
    LD_RED(2),
    SD_RED(3);
    final private int value;

    private AutoPresets(final int value) {
        this.value = value;
    }

    public int getIndex() {
        return this.value;
    }

    public Distance getDistance() {
        return (this.toString().contains("LD")) ? Distance.LONG : Distance.SHORT;
    }
    public Alliance getAlliance() {
        return (this.toString().contains("BLUE")) ? Alliance.BLUE : Alliance.RED;
    }

    public enum Distance {
        SHORT,
        LONG;
    }
    public enum Alliance {
        BLUE,
        RED
    }
}
