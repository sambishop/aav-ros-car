#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <linux/joystick.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static double STEER_MIN_MS = 1.0;
static double STEER_NEUTRAL_MS = 1.5;
static double STEER_MAX_MS = 2.0;
static double DRIVE_MIN_MS = 1.1;
static double DRIVE_NEUTRAL_MS = 1.5;
static double DRIVE_MAX_MS = 1.9;

int openGamepadDevice(const char *gamepadPath)
{
    int gamepadFd = open(gamepadPath, O_RDONLY | O_NONBLOCK);
    if (gamepadFd == -1) {
        fprintf(stderr, "\"%s\" attempting to open \"%s\" for reading\n",
                strerror(errno), gamepadPath);
        exit(1);
    }
    return gamepadFd;
}

bool isAxisEvent(js_event event)
{
    return ((event.type & ~JS_EVENT_INIT) == JS_EVENT_AXIS) && !(~1U & event.number);
}

void processEvent(js_event event, int *xAxis, int *yAxis)
{
    if (event.number & 1) {
        *yAxis = -event.value;
    } else {
        *xAxis = event.value;
    }
}

FILE *openPwmDevice(const char *pwmPath)
{
    FILE *file = fopen(pwmPath, "w");
    if (file == NULL) {
        fprintf(stderr, "\"%s\" attempting to open \"%s\" for reading", strerror(errno), pwmPath);
        exit(1);
    }
    setvbuf(file, NULL, _IOLBF, BUFSIZ);
    return file;
}

void writePwmDuty(FILE *pwmFile, unsigned long long nanoseconds)
{
    fprintf(pwmFile, "%llu\n", nanoseconds);
}

unsigned long long calculateNanos(double min, double max, double neutral, int value)
{
    static const int NS_PER_MS = 1000000;
    if (value < 0) {
        fprintf(stderr, "< 0 %10llu\n", (unsigned long long) (((neutral - min) / 32767 * value + min) * NS_PER_MS));
        return (unsigned long long) (((neutral - min) / 32767 * value + min) * NS_PER_MS);
    } else if (value > 0) {
        fprintf(stderr, "> 0 %10llu\n", (unsigned long long) (((max - neutral) / 32767 * value + neutral) * NS_PER_MS));
        return (unsigned long long) (((max - neutral) / 32767 * value + neutral) * NS_PER_MS);
    } else {
        fprintf(stderr, "  0 %10llu\n", (unsigned long long) (neutral * NS_PER_MS));
        return (unsigned long long) (neutral * NS_PER_MS);
    }
}

void eventLoop(int gamepadFd, FILE *steerFile, FILE *driveFile)
{
    pollfd p;
    p.fd = gamepadFd;
    p.events = POLLIN;
    int xAxis = INT_MAX;
    int yAxis = INT_MAX;

    while (1) {
        js_event event;
        int previousX = xAxis;
        int previousY = yAxis;
        while (read(gamepadFd, &event, sizeof(event)) > 0) {
            if (isAxisEvent(event)) {
                processEvent(event, &xAxis, &yAxis);
            }
        }

        if (errno != EAGAIN) {
            fprintf(stderr, "\"%s\" reading the gamepad device\n", strerror(errno));
            exit(1);
        }

        fprintf(stderr, "[%d, %d]\n", xAxis, yAxis);
        if (xAxis != previousX) {
            writePwmDuty(steerFile, calculateNanos(STEER_MIN_MS, STEER_MAX_MS, STEER_NEUTRAL_MS, xAxis));
        }
        if (yAxis != previousY) {
            writePwmDuty(driveFile, calculateNanos(DRIVE_MIN_MS, DRIVE_MAX_MS, DRIVE_NEUTRAL_MS, yAxis));
        }

        poll(&p, 1, -1);
    }
}

int main(int argc, char **argv)
{
    if (argc != 4) {
        fprintf(stderr, "Usage: gamepad_pwm GAMEPAD_NODE STEER_NODE DRIVE_NODE\n");
        exit(1);
    }
    const char *gamepadPath = argv[1];
    const char *steerPath = argv[2];
    const char *drivePath = argv[3];

    int gamepadFd = openGamepadDevice(gamepadPath);
    FILE *steerFile = openPwmDevice(steerPath);
    FILE *driveFile = openPwmDevice(drivePath);
    eventLoop(gamepadFd, steerFile, driveFile);

    return 0;
}

