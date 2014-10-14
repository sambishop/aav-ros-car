#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

/* LS20031 protocol info:
 *   - NMEA 0183 version 3.01
 *   - 9600 bps, 8 data bits, no parity, 1 stop bit (default)
 *   - 1Hz (default) messages: GGA, GLL, GSA, GSV, RMC, VTG
 */

struct utc_time {
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t centisecond;
};

struct rmc_msg {
    struct utc_time timestamp;
};

FILE *open_device(const char *file)
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

    return fdopen(fd, "w+");
}

int main(int argc, char **argv)
{
    ssize_t line_len;
    char *buf = NULL;
    size_t buf_len = 0;
    FILE *file;

    file = open_device("/dev/ttyAMA0");
    while ((line_len = getline(&buf, &buf_len, file)) != -1) {
        if (strncmp("$GPRMC", buf, 6) == 0) {
            printf("%s", buf);
        } else if (strncmp("$GPGSA", buf, 6) == 0) {
            printf("%s", buf);
        }
    }
}

