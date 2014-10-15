#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "GpsDriver.h"

GpsDriver::GpsDriver(int fd)
{
    /*
    fcntl(fd, F_SETFL, 0);
    struct termios settings;
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

    file = fdopen(fd, "w+");
    // Shut off all sentences except GGA sentences.
    fprintf(file, "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n");
    */
}

char *GpsDriver::readSentence()
{
    sleep(1);
    return strdup("$GPGGA,184430.000,4332.8301,N,11619.8207,W,2,8,1.30,845.0,M,-18.6,M,0000,0000*69\r\n");
    /*
    ssize_t line_len;
    while ((line_len = getline(&buffer, &bufferLength, file)) != -1
            && strncmp("$GPGGA", buffer, 6) != 0)
        ;
    return line_len == -1 ? NULL : buffer;
    */
}

void GpsDriver::writeSentence(const char *sentence)
{
    //fprintf(file, "$%s\r\n", sentence);
}

