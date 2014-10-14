#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

/* LS20031 protocol info:
 *   - NMEA 0183 version 3.01
 *   - 9600 bps, 8 data bits, no parity, 1 stop bit (default)
 *   - 1Hz (default) messages: GGA, GLL, GSA, GSV, RMC, VTG
 */

int open_device(const char *file)
{
    int fd;
    struct termios settings;

    fd = open(file, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        fprintf(stderr, "'%s' attempting to open '%s'\n",
                strerror(errno), file);
    } else {
        fcntl(fd, F_SETFL, 0);
    }

    tcgetattr(fd, &settings);
    cfsetispeed(&settings, B57600);
    cfsetospeed(&settings, B57600);
    settings.c_cflag |= (CLOCAL | CREAD);
    settings.c_cflag &= ~(PARENB | PARODD | CSTOPB | CSIZE | CRTSCTS);
    settings.c_cflag |= CS8;
    settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    settings.c_iflag &= ~(PARMRK | INPCK);
    settings.c_oflag &= ~OPOST;
    tcsetattr(fd, TCSAFLUSH, &settings);

    return fd;
}

int main(int argc, char **argv)
{
    int fd;
    char buf[256];
    ssize_t num_bytes;
    int i;

    fd = open_device("/dev/ttyAMA0");
    while ((num_bytes = read(fd, buf, 256)) != -1) {
        write(1, buf, num_bytes);
        //printf("%d\n", num_bytes);
        /*
        for (i = 0; i < num_bytes; ++i) {
            printf(" %02x", buf[i] & 0xff);
        }
        printf("\n");
        */
    }
}

