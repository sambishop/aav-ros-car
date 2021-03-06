#include <linux/joystick.h>
#include <math.h>
#include <unistd.h>

#include "Gamepad.h"

#define DEADZONE 5

using aav_gamepad_driver::Gamepad;
using aav_gamepad_driver::Position;

Gamepad::Gamepad(int fd) {
  this->fd_ = fd;
  resetTimeout();
}

void Gamepad::resetTimeout() {
  timeout_.tv_sec = 0;
  timeout_.tv_usec = 250000;
}

int Gamepad::waitForUpdate() {
  fd_set set;
  FD_ZERO(&set);
  FD_SET(fd_, &set);
  return select(fd_ + 1, &set, NULL, NULL, &timeout_);
}

Position Gamepad::readPosition() {
  // The kernel returns information regarding the whole gamepad, not
  // just the analog stick we are interested in.  Also, we only get
  // data for one axis at a time.  So the code below does extra
  // bookkeeping to make sure that this function only returns when a
  // value for both axes has been captured or the stick has been
  // moved.
  //
  // When the event number is zero, the value corresponds to the
  // horizontal axis.  (The far left is -32767 and the far right is
  // 32767.)  When the event number is one, the value corresponds to
  // the vertical axis.  (All the way forward is -32767 and all the
  // way back is 32767.)  Note that -32768 is never used in order to
  // keep the axis symmetrical.  Also note that I find it weird to
  // model forward as negative, so I invert the sign of the y value
  // before returning it.
  js_event e;
  static int16_t x = -32768;
  static int16_t y = -32768;
  int16_t old_x = x;
  int16_t old_y = y;

  while (x == -32768 || y == -32768 || (x == old_x && y == old_y)) {
    if (waitForUpdate() == 0) {
      resetTimeout();
      return Position(x, -y);
    }
    read(fd_, &e, sizeof(e));  // TODO check return value
    if ((e.type & ~JS_EVENT_INIT) == JS_EVENT_AXIS) {
      if (e.value >= -DEADZONE && e.value <= DEADZONE) {
        e.value = 0;
      }

      if (e.number == 0) {
        x = e.value;
      } else if (e.number == 1) {
        y = e.value;
      }
    }
  }
  resetTimeout();
  return Position(x, -y);
}

