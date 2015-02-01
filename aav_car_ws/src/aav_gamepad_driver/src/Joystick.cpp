#include <linux/joystick.h>

#include "Joystick.h"

Joystick::Joystick(int fd)
{
    file = fdopen(fd, "r");
}

Event Joystick::readEvent()
{
    js_event event;
    while (true) {
        fread(&event, sizeof(event), 1, file);
        if ((event.type & ~JS_EVENT_INIT) == JS_EVENT_AXIS) {
            if (event.number == 0) {
                return Event(X, event.value);
            } else if (event.number == 1) {
                return Event(Y, event.value);
            }
        }
    }
}

