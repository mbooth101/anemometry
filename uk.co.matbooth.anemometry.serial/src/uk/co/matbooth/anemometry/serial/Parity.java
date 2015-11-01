package uk.co.matbooth.anemometry.serial;

public enum Parity {
    NONE(0),
    ODD(1),
    EVEN(2);

    private final int parity;

    private Parity(final int parity) {
        this.parity = parity;
    }

    public int getParity() {
        return parity;
    }

    public static Parity getDefault() {
        return NONE;
    }
}
