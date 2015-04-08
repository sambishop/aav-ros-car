#ifndef AAV_CONTROL_PATH_POINT_H
#define	AAV_CONTROL_PATH_POINT_H

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

