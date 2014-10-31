#include <errno.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int openDevice(const char *path)
{
    int jsFd = open(path, O_RDONLY | O_NONBLOCK);
    if (jsFd == -1) {
        fprintf(stderr, "\"%s\" attempting to open \"%s\" for reading\n",
                strerror(errno), path);
        exit(1);
    }
    return jsFd;
}

bool isAxisEvent(js_event event)
{
    return ((event.type & ~JS_EVENT_INIT) == JS_EVENT_AXIS) && !(~1U & event.number);
}

void processEvent(js_event event, int *hortizontalAxis, int *verticalAxis)
{
    *((event.number & 1) ? verticalAxis : hortizontalAxis) = event.value;
}

void eventLoop(int jsFd, const char *path)
{
    pollfd p;
    p.fd = jsFd;
    p.events = POLLIN;
    int hortizontalAxis = 0;
    int verticalAxis = 0;

    while (1) {
        js_event event;
        while (read(jsFd, &event, sizeof(event)) > 0) {
            if (isAxisEvent(event)) {
                processEvent(event, &hortizontalAxis, &verticalAxis);
                fprintf(stderr, "[%d, %d]\n", hortizontalAxis, verticalAxis);
            }
        }

        if (errno != EAGAIN) {
            fprintf(stderr, "\"%s\" attempting to read from \"%s\"\n",
                    strerror(errno), path);
            exit(1);
        }

        poll(&p, 1, -1);
    }
}

int main(int argc, char **argv)
{
    if (argc != 3) {
        fprintf(stderr, "Usage: gamepad_pwm GAMEPAD_NODE PWM_NODE\n");
        exit(1);
    }
    const char *path = argv[1];
    int jsFd = openDevice(path);
    eventLoop(jsFd, path);
    return 0;
}

