package uk.co.matbooth.anemometry.serial;

public enum CharSize {
    CHAR5(5),
    CHAR6(6),
    CHAR7(7),
    CHAR8(8);

    private final int size;

    private CharSize(final int size) {
        this.size = size;
    }

    public int getSize() {
        return size;
    }

    public static CharSize getDefault() {
        return CHAR8;
    }
}
