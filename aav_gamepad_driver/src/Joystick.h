#ifndef _JOYSTICK_H_
#define _JOYSTICK_H_

#include <inttypes.h>

struct Position {
  Position(int16_t x, int16_t y) : x(x), y(y) {}
  const int16_t x;
  const int16_t y;
};

class Joystick {
  private:
    int fd;

  public:
    Joystick(int fd);
    Position readPosition();
};

#endif

