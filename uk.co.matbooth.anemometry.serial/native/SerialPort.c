#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <jni.h>

extern int errno;

jint throwIOException(JNIEnv *env)
{
    jclass exClass = (*env)->FindClass(env, "java/io/IOException");
    return (*env)->ThrowNew(env, exClass, strerror(errno));
}

JNIEXPORT jlong JNICALL Java_uk_co_matbooth_anemometry_serial_SerialPort__1open(
        JNIEnv *env, jobject jobj, jstring port, jint baudRate, jint charSize,
        jint parity, jint stopBits)
{
    const char *portName = (*env)->GetStringUTFChars(env, port, NULL);
    int handle = open(portName, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (handle < 0)
    {
        (*env)->ReleaseStringUTFChars(env, port, portName);
        throwIOException(env);
        return handle;
    }
    fcntl(handle, F_SETFL, O_NONBLOCK);

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    tcgetattr(handle, &tty);

    // Set baud rate
    speed_t baud = 0;
    switch (baudRate)
    {
    case 50:
        baud = B50;
        break;
    case 75:
        baud = B75;
        break;
    case 110:
        baud = B110;
        break;
    case 134:
        baud = B134;
        break;
    case 150:
        baud = B150;
        break;
    case 200:
        baud = B200;
        break;
    case 300:
        baud = B300;
        break;
    case 600:
        baud = B600;
        break;
    case 1200:
        baud = B1200;
        break;
    case 1800:
        baud = B1800;
        break;
    case 2400:
        baud = B2400;
        break;
    case 4800:
        baud = B4800;
        break;
    case 9600:
        baud = B9600;
        break;
    case 19200:
        baud = B19200;
        break;
    case 38400:
        baud = B38400;
        break;
    case 57600:
        baud = B57600;
        break;
    case 115200:
        baud = B115200;
        break;
    case 230400:
        baud = B230400;
        break;
    }
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    // Set character size
    tty.c_cflag &= ~CSIZE;
    switch (charSize)
    {
    case 5:
        tty.c_cflag |= CS5;
        break;
    case 6:
        tty.c_cflag |= CS6;
        break;
    case 7:
        tty.c_cflag |= CS7;
        break;
    case 8:
        tty.c_cflag |= CS8;
        break;
    }

    // Set parity
    switch (parity)
    {
    case 0: // None
        tty.c_cflag &= ~(PARENB | PARODD);
        break;
    case 1: // Odd
        tty.c_cflag |= (PARENB | PARODD);
        break;
    case 2: // Even
        tty.c_cflag |= PARENB;
        tty.c_cflag &= ~PARODD;
        break;
    }

    // Set stop bits
    switch (stopBits)
    {
    case 1:
        tty.c_cflag &= ~CSTOPB;
        break;
    case 2:
        tty.c_cflag |= CSTOPB;
        break;
    }

    // Set additional control modes
    tty.c_cflag |= (CREAD | CLOCAL | HUPCL);

    // Set "raw" mode
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL
            | IXON);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ICANON | ISIG | IEXTEN | ECHO | ECHOE | ECHONL);

    // Set control characters
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(handle, TCIFLUSH);
    tcsetattr(handle, TCSANOW, &tty);

    (*env)->ReleaseStringUTFChars(env, port, portName);
    return handle;
}

JNIEXPORT void JNICALL Java_uk_co_matbooth_anemometry_serial_SerialPort__1close(
        JNIEnv *env, jobject jobj, jstring port, jlong handle)
{
    int result = close(handle);
    if (result < 0)
    {
        throwIOException(env);
    }
}

JNIEXPORT jint JNICALL Java_uk_co_matbooth_anemometry_serial_SerialPort__1read(
        JNIEnv *env, jobject jobj, jlong handle, jbyteArray bytes, jint offset,
        jint size)
{
    int avail, recvd = 0;
    ioctl(handle, FIONREAD, &avail);
    if (avail > 0)
    {
        recvd = size < avail ? size : avail;
        jbyte buff[recvd];
        recvd = read(handle, buff, sizeof buff);
        if (recvd > 0)
        {
            (*env)->SetByteArrayRegion(env, bytes, offset, recvd, buff);
        }
    }
    return recvd;
}
