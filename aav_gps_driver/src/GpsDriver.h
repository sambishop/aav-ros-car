#ifndef _GPS_DRIVER_H_
#define _GPS_DRIVER_H_

#include <stdio.h>

class GpsDriver {
    private:
        FILE *file;
        char *buffer;
        size_t bufferLength;

    public:
        GpsDriver(int fd);
        char *readSentence();
        void writeSentence(const char *sentence);
};

#endif

