#ifndef AAV_CONTROL_PATHPOINT_H
#define	AAV_CONTROL_PATHPOINT_H

namespace aav_control
{

struct PathPoint
{
  PathPoint(unsigned int segment, double t) : segment(segment), t(t)
  {}

  unsigned int segment;
  double t;
};

} // end namespace aav_control

#endif

