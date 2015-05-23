#include <fcntl.h>
#include <linux/joystick.h>
#include <unistd.h>

#include "Joystick.h"

Joystick::Joystick(int fd)
{
  int flags = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  this->fd = fd;
}

Position Joystick::readPosition()
{
  static int16_t x = 0;
  static int16_t y = 0;
  js_event event;

  while (read(fd, &event, sizeof(event)) > 0) {
    if ((event.type & ~JS_EVENT_INIT) == JS_EVENT_AXIS) {
      if (event.number == 0) {
        x = event.value;
      } else if (event.number == 1) {
        y = event.value;
      }
    }
  }
  return Position(x, y);
}

