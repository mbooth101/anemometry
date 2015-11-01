package uk.co.matbooth.anemometry.serial;

import java.io.IOException;

/**
 * An immutable object that represents a serial port. It may not be instantiated directly by
 * clients, instead a {@link Builder} object should be used to create connections.
 */
public class SerialPort {

    private static final String LIBRARY_NAME = "serial";

    static {
        try {
            System.loadLibrary(LIBRARY_NAME);
        } catch (UnsatisfiedLinkError e) {
            // TODO
            e.printStackTrace();
        }
    }

    private final String port;
    private final BaudRate baudRate;
    private final CharSize charSize;
    private final Parity parity;
    private final StopBits stopBits;

    private long handle;

    private final Object closeLock = new Object();
    private volatile boolean closed = true;

    /**
     * Constructs an immutable port from the given builder. This should not be called directly by
     * clients, instead use {@link Builder#build()}.
     */
    private SerialPort(final Builder builder) {
        this.port = builder.port;
        this.baudRate = builder.baudRate;
        this.charSize = builder.charSize;
        this.parity = builder.parity;
        this.stopBits = builder.stopBits;
    }

    /**
     * Opens the serial port for reading and writing. Once open, subsequent calls to this method
     * have no effect.
     *
     * @throws IOException
     *             if there was a problem opening the serial port
     */
    public void open() throws IOException {
        synchronized (closeLock) {
            if (!closed) {
                return;
            }
            closed = false;
        }
        handle = _open(port, baudRate.getRate(), charSize.getSize(), parity.getParity(),
                stopBits.getBits());
    }

    /**
     * Closes the serial port for reading and writing and frees any associated system resources.
     * Once closed, subsequent calls to this method have no effect.
     *
     * @throws IOException
     *             if there was a problem closing the serial port
     */
    public void close() throws IOException {
        synchronized (closeLock) {
            if (closed) {
                return;
            }
            closed = true;
        }
        _close(handle);
    }

    public int read(byte[] b, int off, int len) throws IOException {
        int n = _read(handle, b, off, len);
        return n;
    }

    /**
     * This builder gathers various configuration parameters for constructing a {@link SerialPort}.
     * Once configured, the port may be constructed with a call to {@link #build()}. The returned
     * port is immutable.
     */
    public static class Builder {

        private final String port;
        private BaudRate baudRate = BaudRate.getDefault();
        private CharSize charSize = CharSize.getDefault();
        private Parity parity = Parity.getDefault();
        private StopBits stopBits = StopBits.getDefault();

        /**
         * Create a new builder for the given serial port.
         *
         * @param port
         *            the name or full path of a serial port, e.g., "/dev/tty.usbserial" on Mac,
         *            "/dev/ttyUSB0" on Linux or "COM1" on Windows
         */
        public Builder(final String port) {
            this.port = port;
        }

        /**
         * Sets the input and output transmission rate in number of symbols per second. If not
         * specified, the value is set to that returned by {@link BaudRate#getDefault()}.
         *
         * @param baudRate
         *            one of the {@link BaudRate} enumeration values
         * @return the builder for method chaining
         * @throws IllegalArgumentException
         *             if a valid baud rate was not specified
         */
        public Builder baudRate(final BaudRate baudRate) {
            if (baudRate == null) {
                throw new IllegalArgumentException("baudRate must not be null");
            }
            this.baudRate = baudRate;
            return this;
        }

        /**
         * Sets the character size in number of bits per symbol. If not specified, the value is set
         * to that returned by {@link CharSize#getDefault()}.
         *
         * @param charSize
         *            one of the {@link CharSize} enumeration values
         * @return the builder for method chaining
         * @throws IllegalArgumentException
         *             if a valid character size was not specified
         */
        public Builder charSize(final CharSize charSize) {
            if (charSize == null) {
                throw new IllegalArgumentException("charSize must not be null");
            }
            this.charSize = charSize;
            return this;
        }

        /**
         * Sets the parity method used for error checking. If not specified, the value to set to
         * that returned by {@link Parity#getDefault()}.
         *
         * @param parity
         *            one of the {@link Parity} enumeration values
         * @return the builder for method chaining
         * @throws IllegalArgumentException
         *             if a valid parity method was not specified
         */
        public Builder parity(final Parity parity) {
            if (parity == null) {
                throw new IllegalArgumentException("parity must not be null");
            }
            this.parity = parity;
            return this;
        }

        /**
         * Sets the number of stop bits sent after every character. If not specified, the value to
         * set to that returned by {@link StopBits#getDefault()}.
         *
         * @param stopBits
         *            one of the {@link StopBits} enumeration values
         * @return the builder for method chaining
         * @throws IllegalArgumentException
         *             if a valid number of stop bits was not specified
         */
        public Builder stopBits(final StopBits stopBits) {
            if (stopBits == null) {
                throw new IllegalArgumentException("stopBits must not be null");
            }
            this.stopBits = stopBits;
            return this;
        }

        /**
         * Constructs a new port from this builder.
         *
         * @return a new, immutable {@link SerialPort}
         */
        public SerialPort build() {
            return new SerialPort(this);
        }
    }

    private native long _open(String port, int baudRate, int charSize, int parity, int stopBits)
            throws IOException;

    private native void _close(long handle) throws IOException;

    private native int _read(long handle, byte[] b, int off, int len);
}
