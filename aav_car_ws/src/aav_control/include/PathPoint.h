#ifndef AAV_CONTROL_PATHPOINT_H
#define	AAV_CONTROL_PATHPOINT_H

namespace aav_control
{

class PathPoint
{
public:
  PathPoint(unsigned int segment, double t);
  unsigned int getSegment() const;
  double getT() const;

private:
  unsigned int segment;
  double t;
};

} // end namespace aav_control

#endif

