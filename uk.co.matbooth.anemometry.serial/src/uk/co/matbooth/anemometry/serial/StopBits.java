package uk.co.matbooth.anemometry.serial;

public enum StopBits {
    STOP1(1),
    STOP2(2);

    private final int bits;

    private StopBits(final int bits) {
        this.bits = bits;
    }

    public int getBits() {
        return bits;
    }

    public static StopBits getDefault() {
        return STOP1;
    }
}
