#ifndef AAV_GAMEPAD_DRIVER_GAMEPAD_H
#define AAV_GAMEPAD_DRIVER_GAMEPAD_H

#include <inttypes.h>

namespace aav_gamepad_driver {

// This struct represents the position of the left analog stick on
// the gamepad.  The x member represents the horizontal axis.  All
// the way left is -32767 and all the way right is 32767.  The y
// member represents the vertical axis.  All the way forward is
// 32767 and all the way back is -32767.  (Note that 32768 is never
// used in order to keep the axes symmetrical.)  The neutral
// position should be (0, 0), though that is only true if the
// gamepad has been calibrated.
struct Position {
  Position(int16_t x, int16_t y) : x(x), y(y) {}
  const int16_t x;
  const int16_t y;
};

class Gamepad {
private:
  int fd_;

public:
  Gamepad(int fd);
  Position readPosition();
};

}

#endif

