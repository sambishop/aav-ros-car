#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

FILE *open_device(const char *path)
{
    int fd;
    struct termios settings;

    fd = open(path, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        fprintf(stderr, "'%s' attempting to open '%s'\n",
                strerror(errno), path);
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
    settings.c_iflag &= ~(PARMRK | INPCK | ICRNL);
    settings.c_oflag &= ~OPOST;
    tcsetattr(fd, TCSAFLUSH, &settings);

    return fdopen(fd, "w+");
}

void request_only_gga_messages(FILE *file)
{
    fprintf(file, "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n");
}

void bump_speed_to_115200(FILE *file)
{
    fprintf(file, "$PMTK251,115200*1F\r\n");
    fflush(file);
    int fd = fileno(file);
    struct termios settings;
    tcgetattr(fd, &settings);
    cfsetispeed(&settings, B115200);
    cfsetospeed(&settings, B115200);
    tcsetattr(fd, TCSAFLUSH, &settings);
    fprintf(file, "$PMTK220,100*2F\r\n");
}

int main(int argc, char **argv)
{
    ssize_t line_len;
    char *buf = NULL;
    size_t buf_len = 0;
    FILE *file;

    file = open_device("/dev/ttyO1");
    request_only_gga_messages(file);
    bump_speed_to_115200(file);
    while ((line_len = getline(&buf, &buf_len, file)) != -1) {
        if (strncmp("$GPGGA", buf, 6) == 0) {
            printf("%s", buf);
        } else {
            printf("IGNORED: [%zd:%zd] %s", line_len, buf_len, buf);
        }
    }

    return 0;
}

/* Example NMEA sentence:
 * $GPGGA,184430.000,4332.8301,N,11619.8207,W,2,8,1.30,845.0,M,-18.6,M,0000,0000*69
 */

