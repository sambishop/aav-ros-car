#ifndef _JOYSTICK_H_
#define _JOYSTICK_H_

#include <stdio.h>
#include <inttypes.h>

enum Axis {
    X, Y
};

struct Event {
    Event(Axis _axis, int16_t _value) : axis(_axis), value(_value) {}
    const Axis axis;
    const int16_t value;
};

class Joystick {
    private:
        FILE *file;

    public:
        Joystick(int fd);
        Event readEvent();
};

#endif

